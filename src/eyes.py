# Copyright (C) 2026 Pau Anguera Comas
#
# Aquest programa és programari lliure: podeu redistribuir-lo i/o
# modificar-lo sota els termes de la Llicència Pública General de GNU
# publicada per la Free Software Foundation, en la versió 3.
#
# Es distribueix amb l'esperança que sigui útil, però SENSE CAP GARANTIA.
# Vegeu la Llicència Pública General de GNU per a més detalls.
# <https://www.gnu.org/licenses/>

"""
eyes.py — Control de les matrius LED 8x8 RGB WS2811 (ulls del robot).
Equivalent a eyes.cpp del codi Arduino/ESP32.

FastLED → rpi_ws281x (timing per HARDWARE, PWM1). Els LEDs son a GPIO13 (pont
fisic des de GPIO9, que no te PWM/PCM). pigpio (resolucio 1µs) NO podia fer el
timing del WS2811 -> el bit "0" sortia com "1" (tot blanc); rpi_ws281x ho fa be.

REQUISITS:
  - Llibreria: sudo pip install rpi_ws281x
  - rpi_ws281x per PWM necessita ROOT -> executar el programa amb sudo.
  - Pont GPIO13→GPIO9 fet; GPIO9 es deixa en INPUT (ho fa eyes_setup) per no
    fer conflicte amb GPIO13.
"""

import time
import math
import threading
import pigpio
from rpi_ws281x import PixelStrip, Color, ws

from pins import LED_DATA, LED_DATA_OLD

# Configuracio de la tira WS2811 (rpi_ws281x, timing per hardware via PWM1).
# LED_DATA = GPIO13 (canal PWM 1). Si els colors R/G surten canviats -> RGB.
_LED_FREQ_HZ  = 800000
_LED_DMA      = 10
_LED_INVERT   = False
_LED_CHANNEL  = 1                     # GPIO13 = PWM1 -> canal 1
_LED_STRIP    = ws.WS2811_STRIP_GRB   # ordre GRB (com FastLED)
_strip        = None                  # instancia PixelStrip (creada a eyes_setup)

# ==================
# Constants
# ==================

ROW_NUM  = 8
COL_NUM  = 8
NUM_LEDS = ROW_NUM * COL_NUM * 2   # 128 LEDs (2 matrius 8x8)

# Intensitat global dels ulls (0-255). ÚNIC lloc per canviar-la: tot eyes.py
# (i qualsevol test que usi aquest mòdul) en depèn. Intensitat CONSTANT (no
# respira): tots els estats es mostren a EYES_BRIGHTNESS.
EYES_BRIGHTNESS = 50

# Colors predefinits (R, G, B)
WHITE  = (255, 255, 255)
RED    = (255,   0,   0)
GREEN  = (  0, 255,   0)
BLUE   = (  0,   0, 255)
YELLOW = (255, 255,   0)   # groc pur
ORANGE = (255,  80,   0)
PURPLE = (140,   0, 255)   # lila
CYAN   = (  0, 255, 255)   # Color del mode gestos
BLACK  = (  0,   0,   0)

# ==================
# Formes dels ulls (índexs dels LEDs actius)
# ==================

class EyeShape:
    """Conjunt de LEDs que formen una expressió dels ulls."""
    def __init__(self, leds: list):
        self.leds = leds
        self.len  = len(leds)


EYES_OPEN = EyeShape([
    102, 89, 38, 25, 106, 101, 90, 85, 42, 37, 26, 21,
    107, 100, 91, 84, 43, 36, 27, 20, 108, 99, 92, 83,
    44, 35, 28, 19, 109, 98, 93, 82, 45, 34, 29, 18,
    97, 94, 33, 30
])

EYES_FW = EyeShape([
    103, 88, 39, 24, 105, 102, 89, 86, 41, 38, 25, 22,
    117, 106, 101, 90, 85, 74, 53, 42, 37, 26, 21, 10,
    123, 116, 100, 91, 75, 68, 59, 52, 36, 27, 11, 4,
    99, 92, 35, 28, 98, 93, 34, 29, 97, 94, 33, 30, 96,
    95, 32, 31
])

EYES_DOWN = EyeShape([97, 94, 33, 30])

# Nova forma per al mode gestos: marc extern dels dos ulls (expressió "atenta")
EYES_GESTURE = EyeShape([
    96, 97, 98, 99, 100, 101, 102, 103,
    104, 111, 112, 119, 120, 127,
    31, 32, 33, 34, 35, 36, 37, 38, 39,
    0, 7, 8, 15, 16, 23
])

# ==================
# Estat global
# ==================

_pi: pigpio.pi = None
_leds          = [[0, 0, 0] for _ in range(NUM_LEDS)]
_leds_lock     = threading.Lock()

_update_stop   = threading.Event()
_update_thread: threading.Thread = None

# Mode de render del thread: "static" (renderitza _leds amb l'efecte respirar)
# o "loading" (animació de càrrega: franja que llisca d'esquerra a dreta).
_anim_mode = "static"
_last_frame = None       # últim fotograma pintat, per no repintar sense canvis
_LOADING_WIDTH = 3     # nombre de columnes enceses de la franja de càrrega
_blink_color = (255, 255, 255)   # color del mode "blink_color" (esperant confirmació)

# ==================
# WS2811 via rpi_ws281x (timing per hardware, PWM1 a GPIO13)
# ==================
# pigpio (resolucio 1µs) NO pot codificar el bit "0" del WS2811 (sortia com "1"
# -> tot blanc). rpi_ws281x fa el timing per hardware (PWM/DMA) -> correcte.

def _eyes_show(brightness: int):
    """Renderitza l'estat actual de _leds amb la brillantor indicada."""
    if _strip is None:
        return
    _strip.setBrightness(brightness)          # brillantor global (efecte respirar)
    for i, (r, g, b) in enumerate(_leds):
        _strip.setPixelColor(i, Color(r, g, b))
    _strip.show()


# ==================
# Setup i cleanup
# ==================

def eyes_setup(pi: pigpio.pi):
    """
    Inicialitza la tira WS2811 (rpi_ws281x a GPIO13) i arrenca el thread de
    parpelleig. 'pi' (pigpio) s'usa nomes per deixar GPIO9 (pin antic) en INPUT,
    perque no lluiti amb GPIO13 a traves del pont fisic.
    """
    global _pi, _strip, _update_thread

    _pi = pi

    # Deixa el pin antic (GPIO9) en alta impedancia -> nomes GPIO13 mana la linia.
    if pi is not None:
        pi.set_mode(LED_DATA_OLD, pigpio.INPUT)

    _strip = PixelStrip(NUM_LEDS, LED_DATA, _LED_FREQ_HZ, _LED_DMA,
                        _LED_INVERT, EYES_BRIGHTNESS, _LED_CHANNEL, _LED_STRIP)
    _strip.begin()

    _update_stop.clear()
    _update_thread = threading.Thread(
        target=_task_update_leds, daemon=True, name="eyes"
    )
    _update_thread.start()


def eyes_cleanup():
    """
    Atura el thread de parpelleig i apaga els LEDs.

    Després d'enviar l'apagada cal ESPERAR abans de continuar: els WS2811
    conserven l'últim color encara que se'ls talli el senyal, i el senyal es
    genera per DMA, o sigui que si el procés acaba just després del show() hi
    ha trames que no arriben a tota la tira. El símptoma és que queden uns
    quants LED encesos després d'apagar el robot.
    """
    _update_stop.set()
    if _update_thread:
        _update_thread.join(timeout=1.0)
    with _leds_lock:
        for i in range(NUM_LEDS):
            _leds[i] = [0, 0, 0]
        _eyes_show(0)
    time.sleep(0.2)      # temps perquè el DMA acabi d'enviar la trama


# ==================
# Thread de parpelleig (equivalent a task_update_leds del FreeRTOS)
# ==================

def _task_update_leds():
    """
    Bucle continu de render dels ulls.
      - "static": mostra _leds a intensitat CONSTANT (EYES_BRIGHTNESS), sense
        respirar. La majoria d'estats (idle, colors, tick, creu).
      - "loading" / "blink_color": animacions (només quan s'activen).
    """
    global _last_frame
    frame = 0
    while not _update_stop.is_set():
        # Mode "loading": animació de càrrega (franja que llisca esq -> dreta)
        if _anim_mode == "loading":
            _render_loading(frame)
            _last_frame = None
            frame += 1
            time.sleep(0.06)
            continue

        # Mode "blink_color": parpelleig del color (esperant confirmació)
        if _anim_mode == "blink_color":
            _render_blink(frame)
            _last_frame = None
            frame += 1
            time.sleep(0.05)
            continue

        # Mode "static": brillantor CONSTANT, sense respirar.
        #
        # Només es repinta si els LEDs han canviat des de l'últim cop. Abans es
        # renderitzaven els 128 LEDs vint vegades per segon encara que no hagués
        # canviat res, i en una Raspberry d'un sol nucli això li robava temps al
        # generador de polsos dels motors.
        with _leds_lock:
            actual = tuple(tuple(px) for px in _leds)
            if actual != _last_frame:
                _eyes_show(EYES_BRIGHTNESS)
                _last_frame = actual
        time.sleep(0.05)


def _render_loading(frame: int):
    """
    Franja de _LOADING_WIDTH columnes que llisca d'ESQUERRA a DRETA (vista des
    del davant) a través dels 2 ulls: 16 columnes (0-7 ull esquerre, 8-15 dret),
    files 2-5 (ben visible), amb wrap-around continu.
    """
    pos = frame % 16
    lit = set()
    for k in range(_LOADING_WIDTH):
        c = (pos + k) % 16
        eye = 0 if c < 8 else 1
        col = c % 8
        for row in range(2, 6):
            lit.add(_xy_index(eye, col, row))
    with _leds_lock:
        for i in range(NUM_LEDS):
            _leds[i] = [255, 255, 255] if i in lit else [0, 0, 0]
        _eyes_show(EYES_BRIGHTNESS)   # brillantor fixa


_BLINK_STEPS = 32   # frames per cicle del pols (~1,6s a 0,05s/frame)


def _render_blink(frame: int):
    """
    Fade de _blink_color amb un pols SUAU (sinus): la intensitat puja i baixa
    entre 0 i EYES_BRIGHTNESS en un cicle fix. Estat "esperant confirmació".
    """
    level  = (1 - math.cos(2 * math.pi * frame / _BLINK_STEPS)) / 2   # 0->1->0 suau
    bright = int(EYES_BRIGHTNESS * level)
    r, g, b = _blink_color
    with _leds_lock:
        for i in range(NUM_LEDS):
            _leds[i] = [r, g, b]
        _eyes_show(bright)


# ==================
# Animacions (equivalent a les funcions de eyes.cpp)
# ==================

def eyes_turn_off():
    """Apaga tots els LEDs amb un fos progressiu."""
    for _ in range(50):
        with _leds_lock:
            for i in range(NUM_LEDS):
                r, g, b = _leds[i]
                _leds[i] = [int(r * 245 / 255),
                             int(g * 245 / 255),
                             int(b * 245 / 255)]
        time.sleep(0.01)
    with _leds_lock:
        for i in range(NUM_LEDS):
            _leds[i] = [0, 0, 0]


def eyes_turn_on(shape: EyeShape, color: tuple,
                 repeat: int = 1, forward: bool = True):
    """
    Encén els LEDs d'una forma un per un, amb animació.
    shape:   forma a dibuixar (EYES_OPEN, EYES_FW, EYES_DOWN…)
    color:   color RGB com a tupla (r, g, b)
    repeat:  nombre de vegades que es repeteix l'animació
    forward: True = ordre normal, False = ordre invers
    """
    r, g, b = color
    for rep in range(repeat):
        eyes_turn_off()
        for i in range(shape.len):
            idx = shape.leds[i if forward else (shape.len - 1 - i)]
            with _leds_lock:
                _leds[idx] = [r, g, b]
            time.sleep(0.008)

        if rep < repeat - 1:
            for i in range(shape.len):
                idx = shape.leds[i if forward else (shape.len - 1 - i)]
                with _leds_lock:
                    _leds[idx] = [0, 0, 0]
                time.sleep(0.008)


# ==================
# Animacions noves — mode gestos (TFG)
# ==================

def eyes_mode_switch(gestos: bool = True):
    """
    Senyal visual que el robot ha llegit el canvi de mode: dos parpelleigs en
    CIAN, el color del mode gestos.

    És el mateix color en tots dos sentits del canvi. El senyal només ha de
    comunicar "t'he entès"; en quin mode ha quedat ja es veu a l'estat de
    repòs que ve tot seguit.
    """
    global _anim_mode
    _anim_mode = "static"          # atura qualsevol animació en curs
    r, g, b = CYAN
    for _ in range(2):
        with _leds_lock:
            for i in range(NUM_LEDS):
                _leds[i] = [r, g, b]
        time.sleep(0.14)
        with _leds_lock:
            for i in range(NUM_LEDS):
                _leds[i] = [0, 0, 0]
        time.sleep(0.14)


def eyes_gesture_mode_on():
    """
    Animació d'activació del mode gestos.
    Parpelleig doble en cian per indicar que el robot escolta gestos.
    """
    eyes_turn_on(EYES_OPEN, CYAN, repeat=2)


def eyes_gesture_mode_off():
    """
    Animació de desactivació del mode gestos.
    Torna als ulls oberts en blanc.
    """
    eyes_turn_off()
    eyes_turn_on(EYES_OPEN, WHITE)


def eyes_listening():
    """
    Expressió "escoltant": marc extern dels ulls en cian.
    Es mostra mentre el robot espera un gest.
    """
    eyes_turn_on(EYES_GESTURE, CYAN)


# ==================
# Estats per bitmap (mapatge confirmat: index = ull*64 + fila*8 + columna)
# ==================
# Formes definides amb bitmaps 8x8 ('#'=encès, '.'=apagat), dibuixades a les 2
# matrius. Escriuen directament _leds; el thread de parpelleig (respirar) les
# renderitza. Instantani (sense animació), pensat per als ESTATS del robot.

def _xy_index(eye: int, col: int, row: int) -> int:
    return eye * 64 + row * 8 + col


def _bitmap_indices(bitmap) -> set:
    idxs = set()
    for eye in (0, 1):
        for row in range(8):
            for col in range(8):
                if bitmap[row][col] == "#":
                    idxs.add(_xy_index(eye, col, row))
    return idxs


def _set_bitmap(bitmap, color: tuple):
    """Pinta 'color' als '#' del bitmap (a les 2 matrius); la resta, apagat."""
    global _anim_mode
    _anim_mode = "static"          # atura qualsevol animació (p. ex. loading)
    on = _bitmap_indices(bitmap)
    r, g, b = color
    with _leds_lock:
        for i in range(NUM_LEDS):
            _leds[i] = [r, g, b] if i in on else [0, 0, 0]


# Bitmaps dels estats
_BM_FILL = ["########"] * 8            # ulls plens (accions)

_BM_IDLE = [                           # cian amb pupil·la 4x4 al centre (repòs)
    "..####..",
    ".######.",
    "##....##",
    "##....##",
    "##....##",
    "##....##",
    ".######.",
    "..####..",
]

_BM_BLINK = [                          # ratlla horitzontal (no-preparat/arrencant)
    "........",
    "........",
    "........",
    "########",
    "########",
    "........",
    "........",
    "........",
]


def eyes_fill(color: tuple):
    """Omple els dos ulls del color donat. Per a les ACCIONS del robot."""
    _set_bitmap(_BM_FILL, color)


def eyes_idle():
    """Estat en REPÒS (esperant acció): ull cian amb pupil·la 4x4."""
    _set_bitmap(_BM_IDLE, CYAN)


def eyes_blink():
    """
    Estat NO-PREPARAT / arrencant (o cas no previst): animació de CÀRREGA —
    una franja blanca que llisca d'esquerra a dreta a través dels dos ulls.
    """
    global _anim_mode
    _anim_mode = "loading"    # el thread de render fa l'animació


def eyes_blink_color(color: tuple):
    """Parpelleja el color donat. Estat "esperant confirmació" del gest."""
    global _anim_mode, _blink_color
    _blink_color = color
    _anim_mode = "blink_color"


# Bitmaps de confirmació
_BM_TICK = [                           # tick (verd) — versió gran/bold
    "......##",
    ".....###",
    "....###.",
    "##.###..",
    "#####...",
    ".####...",
    "..##....",
    "........",
]

_BM_CROSS = [                          # creu (vermell)
    "#......#",
    "##....##",
    ".##..##.",
    "..####..",
    "..####..",
    ".##..##.",
    "##....##",
    "#......#",
]


def eyes_tick(color: tuple = GREEN):
    """Tick verd: comanda CONFIRMADA."""
    _set_bitmap(_BM_TICK, color)


def eyes_cross(color: tuple = RED):
    """Creu vermella: comanda CANCEL·LADA."""
    _set_bitmap(_BM_CROSS, color)
