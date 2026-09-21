"""
blocks.py — Lectura del sensor de color TCS34725 i servo d'expulsió de blocs.
Equivalent a blocks.cpp del codi Arduino/ESP32.

Sensor de color TCS34725 al bus I2C 1 (hardware GPIO2/3, adreca 0x29;
compartit amb gestos i ADS1115). Servo d'expulsio via pigpio (SERVO_PWM).
"""

import time
import pigpio
import adafruit_extended_bus
import adafruit_tcs34725

from pins import SERVO_PWM

# ==================
# IDs de color
# ==================

BK = 0   # Negre (no reconegut)
RD = 1   # Vermell  → avançar
GN = 2   # Verd     → girar dreta
BU = 3   # Blau     → girar esquerra
YE = 4   # Groc     → xuclar líquid
OG = 5   # Taronja  → buidar líquid
VT = 6   # Violeta  → sorpresa

NUM_COLORS = 7

# Taula de colors de referència (valors RGB 0–255 mesurats amb el sensor al Pi,
# integration_time=50ms, gain=4). Recalibrat 2026-06.
_COLORS = [
    {"name": "BK", "r":  11, "g":  22, "b":  12},   # sense bloc/ambient (mitjana, forat tapat)
    {"name": "RD", "r":  90, "g":   4, "b":   3},
    {"name": "GN", "r":   8, "g":  45, "b":   8},
    {"name": "BU", "r":   2, "g":  18, "b":  42},
    {"name": "YE", "r":  32, "g":  27, "b":   1},
    {"name": "OG", "r":  78, "g":   7, "b":   2},
    {"name": "VT", "r":  14, "g":  16, "b":  20},
]

# ==================
# Paràmetres del servo
# ==================

# Valors en µs de pulse width (pigpio set_servo_pulsewidth).
# Conversió des de l'ESP32 (16 bits, 50Hz): valor/65535 * 20000µs
# MIN  = 3277/65535 * 20000 ≈ 1000µs
# MAX  = 8000/65535 * 20000 ≈ 2440µs
# EJECT= 6450/65535 * 20000 ≈ 1968µs
MIN_SERVO_US   = 1000   # µs → ~0°
MAX_SERVO_US   = 2440   # µs → ~180°
OPEN_POSITION  = 2050   # µs — posicio oberta/repos (calibrat 2026-07 al robot)
EJECT_POSITION = 1550   # µs — posicio d'expulsio del bloc (calibrat 2026-07)
# µs per iteració d'1 ms. El valor heretat era 3, amb el comentari
# "equivalent a l'increment de 10 de l'ESP32" — però la conversió estava
# malament: l'ESP32 movia el servo en GRAUS i aquí es mou en MICROSEGONS, i
# 10 graus són uns 80 µs. Amb 3 el moviment quedava molt més lent del que
# havia de ser, sobretot perquè cada sleep(0.001) real triga bastant més d'un
# mil·lisegon i l'error s'acumula al llarg de centenars d'iteracions.
_INCREMENT_US  = 20

# ==================
# Instàncies globals
# ==================

_pi: pigpio.pi           = None
_color_sensor            = None
_current_servo_pos: int  = OPEN_POSITION


# ==================
# Setup
# ==================

def blocks_setup(pi: pigpio.pi):
    """
    Inicialitza el servo i el sensor de color TCS34725.
    El servo arranca en OPEN_POSITION.
    """
    global _pi, _color_sensor, _current_servo_pos

    _pi = pi

    # Servo
    pi.set_mode(SERVO_PWM, pigpio.OUTPUT)
    _current_servo_pos = OPEN_POSITION
    pi.set_servo_pulsewidth(SERVO_PWM, _current_servo_pos)

    # Sensor de color TCS34725 al bus I2C 1 (hardware GPIO2/3, 0x29)
    i2c = adafruit_extended_bus.ExtendedI2C(1)
    _color_sensor = adafruit_tcs34725.TCS34725(i2c)
    _color_sensor.integration_time = 100   # ms (pujat de 50: mes senyal, menys soroll)
    _color_sensor.gain = 4                 # 4x (equivalent a TCS34725::Gain::X04)


# ==================
# Servo
# ==================

def servo_move_to(target_us: int):
    """
    Mou el servo fins a target_us de forma suau.
    Incrementa/decrementa _INCREMENT_US cada mil·lisegon.
    """
    global _current_servo_pos

    if not (MIN_SERVO_US <= target_us <= MAX_SERVO_US):
        return

    while True:
        if _current_servo_pos < target_us - _INCREMENT_US:
            _current_servo_pos += _INCREMENT_US
        elif _current_servo_pos > target_us + _INCREMENT_US:
            _current_servo_pos -= _INCREMENT_US
        else:
            _current_servo_pos = target_us

        _pi.set_servo_pulsewidth(SERVO_PWM, _current_servo_pos)
        time.sleep(0.001)

        if _current_servo_pos == target_us:
            return


# ==================
# Sensor de color
# ==================

# Llindar de classificacio en espai HUE normalitzat (fraccions x1000, Manhattan).
# Treballar amb fraccions r/(r+g+b) fa la deteccio independent de la BRILLANTOR
# (posicio del bloc, distancia, llum), cosa que abans confonia vermell/taronja
# i violeta/ambient amb valors absoluts.
MAX_DIFFERENCE = 200
MIN_TOTAL      = 10     # si r+g+b < aixo, massa fosc per fiar-se -> BK


def _normalize(r: int, g: int, b: int) -> tuple:
    total = r + g + b
    if total <= 0:
        return (0.0, 0.0, 0.0)
    return (r / total, g / total, b / total)


# Pre-calcula les fraccions normalitzades de cada color de referencia.
for _ref in _COLORS:
    _ref["nr"], _ref["ng"], _ref["nb"] = _normalize(_ref["r"], _ref["g"], _ref["b"])


def read_color_raw() -> tuple:
    """Retorna (r, g, b) en bytes 0-255 sense classificació. Útil per calibrar."""
    try:
        return _color_sensor.color_rgb_bytes
    except Exception:
        return (0, 0, 0)


def classify_color(r: int, g: int, b: int) -> int:
    """
    Classifica un (r,g,b) comparant el HUE normalitzat amb la taula _COLORS.
    Independent de la brillantor. Retorna l'ID del color mes proper, o BK si
    es massa fosc (MIN_TOTAL) o cap referencia supera el llindar.
    """
    total = r + g + b
    if total < MIN_TOTAL:
        return BK
    nr, ng, nb = r / total, g / total, b / total

    best_diff  = MAX_DIFFERENCE
    best_color = BK
    for color_id, ref in enumerate(_COLORS):
        diff = (abs(nr - ref["nr"]) +
                abs(ng - ref["ng"]) +
                abs(nb - ref["nb"])) * 1000
        if diff < best_diff:
            best_diff  = diff
            best_color = color_id
    return best_color


SAMPLES_PER_READ = 5   # lectures que es promitgen per decidir el color


def read_block_color(samples: int = SAMPLES_PER_READ) -> int:
    """
    Llegeix el sensor 'samples' cops, fa la MITJANA dels RGB i classifica.
    Promitjar redueix el soroll que de tant en tant feia confondre el vermell
    amb el taronja (es separen pel canal verd, 4-5 vs 6-7).
    """
    sr = sg = sb = 0.0
    n = 0
    # El sensor nomes actualitza cada integration_time; cal esperar entre mostres
    # perque siguin INDEPENDENTS (si no, promitjar el mateix valor no fa res).
    wait = (_color_sensor.integration_time + 10) / 1000.0   # s
    for i in range(samples):
        try:
            r, g, b = _color_sensor.color_rgb_bytes
        except Exception:
            r = None
        if r is not None:
            sr += r
            sg += g
            sb += b
            n += 1
        if i < samples - 1:
            time.sleep(wait)
    if n == 0:
        return BK
    return classify_color(sr / n, sg / n, sb / n)
