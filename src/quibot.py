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
quibot.py — Programa principal del robot QuiBot H2O.
Inicialitza tots els mòduls, executa el homing i arrenca els threads.
Equivalent a QuiBot.ino del codi Arduino/ESP32.

Threads permanents:
  - task_read_blocks   → llegeix blocs i executa accions (aquest fitxer)
  - task_read_gestures → llegeix gestos i executa accions (aquest fitxer)
  - _stepper_loop      → genera polsos STEP dels motors (motion.py)
  - _task_update_leds  → parpelleig dels LEDs (eyes.py)
  - _poll_loop         → polling del sensor de gestos (gesture.py)
"""

import time
import threading
import signal
import sys
import pigpio

import motion
from motion import (
    motion_setup, motion_cleanup,
    arms_home, syringe_home, enable_arms, move_arms_up, ON,
    task_move_to, task_move_backward, task_rotate,
    task_take_or_leave_something, task_idle,
    distance_to_object,
    CROSSING, TAKE, LEAVE, CW, CCW,
)
from blocks import (
    blocks_setup,
    read_block_color, servo_move_to,
    OPEN_POSITION, EJECT_POSITION,
    BK, RD, GN, BU, YE, OG, VT,
)
from eyes import (
    eyes_setup, eyes_cleanup,
    eyes_turn_on, eyes_turn_off,
    eyes_gesture_mode_on, eyes_gesture_mode_off,
    eyes_fill, eyes_idle, eyes_blink,
    eyes_blink_color, eyes_tick, eyes_cross, eyes_mode_switch,
    EYES_OPEN, EYES_FW, EYES_DOWN,
    RED, GREEN, BLUE, YELLOW, ORANGE, CYAN, PURPLE,
)
from gesture import (
    gesture_setup, gesture_cleanup,
    read_gesture,
    GS_NONE, GS_FORWARD, GS_LEFT, GS_RIGHT,
    GS_UP, GS_DOWN, GS_CLOCKWISE, GS_ANTICLOCKWISE, GS_WAVE,
)

# ==================
# Colors addicionals (equivalents CRGB del C++)
# ==================

GRAY     = (128, 128, 128)   # CRGB::Gray     → estat normal
DARK_RED = (139,   0,   0)   # CRGB::DarkRed  → avançar

# ==================
# Timeouts
# ==================

INSERT_BLOCK_MS = 2.0   # s — espera que l'infant insereixi el bloc
EJECT_BLOCK_MS  = 2.0   # s — espera que el bloc caigui
CHECK_COLOR_MS  = 1.0   # s — interval entre lectures de color

# Interval de sondeig del sensor de gestos. Amb el robot aturat ha de ser prou
# curt perquè els gestos responguin de seguida; movent-se, el que importa és
# no robar CPU al generador de polsos, i de tota manera l'acció en curs no es
# pot interrompre amb un gest.
GESTURE_POLL_S        = 0.05
GESTURE_POLL_MOVING_S = 0.30

# Gestos: temps després d'una acció abans de tornar a escoltar (evita rebots)
GESTURE_COOLDOWN_S   = 1.2
# Temps màxim per confirmar/cancel·lar una comanda amb un gest
CONFIRM_TIMEOUT_S    = 10.0
# Durada de mostrar el tick/creu abans d'executar/cancel·lar
CONFIRM_FEEDBACK_S   = 1.6

# ==================
# Estat global
# ==================

_pi: pigpio.pi = None

# Mutex que evita que tasca de blocs i tasca de gestos llancin accions simultànies.
# En el C++ original compartien TaskHandle sense mutex explícit (possible race condition).
# Aquí ho fem correctament.
_action_lock = threading.Lock()

_shutdown_event      = threading.Event()
_gesture_mode_active = False          # False = mode blocs, True = mode gestos
_mode_lock           = threading.Lock()


# ==================
# Helper d'execució d'accions
# ==================

_busy = False


def _execute_action(fn, *args, **kwargs):
    """
    Adquireix el lock d'acció, executa fn(*args, **kwargs) de forma bloquejant
    i l'allibera. Garanteix que mai s'executen dues accions simultànies.
    """
    global _busy
    with _action_lock:
        _busy = True
        try:
            fn(*args, **kwargs)
        finally:
            _busy = False


# ==================
# Pont amb el servidor web
# ==================

class _RobotBridge:
    """
    Dona al servidor web accés controlat a l'estat del programa. S'injecta a
    webserver.bind() per no crear una dependència circular entre els mòduls.

    La web té MÉS AUTORITAT que els gestos i els blocs: pot canviar de mode i
    aturar el robot. Les accions de moviment, però, passen pel mateix lock que
    la resta, de manera que mai es trepitgen entre elles.
    """

    def execute_action(self, fn, *args, **kwargs):
        _execute_action(fn, *args, **kwargs)

    def is_busy(self):
        return _busy

    def get_mode(self):
        with _mode_lock:
            return _gesture_mode_active

    def set_mode(self, gestos: bool):
        global _gesture_mode_active
        with _mode_lock:
            canvia = _gesture_mode_active != gestos
            _gesture_mode_active = gestos
        if canvia:
            eyes_mode_switch(gestos)   # mateix senyal que en canviar-lo amb el wave
            eyes_idle()

    def request_shutdown(self):
        _shutdown_event.set()


# ==================
# Tasca de blocs
# ==================

def task_read_blocks():
    """
    Llegeix blocs contínuament, executa l'acció corresponent al color i expulsa el bloc.
    Equivalent a task_read_blocks() del FreeRTOS.
    """
    while not _shutdown_event.is_set():
        eyes_state = False
        eyes_idle()                     # repòs: ull cian amb pupil·la, esperant bloc

        # Obre el servo per permetre la inserció del bloc
        servo_move_to(OPEN_POSITION)
        time.sleep(INSERT_BLOCK_MS)

        # Espera que hi hagi un bloc i llegeix el seu color
        color_id = BK
        while not _shutdown_event.is_set():
            if distance_to_object() < 80:
                # Objecte a menys de 80mm — esperem que es retiri
                if not eyes_state:
                    eyes_idle()
                    eyes_state = True
            else:
                color_id = read_block_color()
                if eyes_state:
                    eyes_idle()
                    eyes_state = False
                if color_id != BK:
                    break
            # Mentre el robot es mou no cal llegir el color: ningú pot
            # inserir un bloc i la lectura I²C frenaria el moviment.
            while motion.moving and not _shutdown_event.is_set():
                time.sleep(0.2)
            time.sleep(CHECK_COLOR_MS)

        # Si estem en mode gestos, ignora el bloc
        with _mode_lock:
            if _gesture_mode_active:
                continue

        # Executa l'acció corresponent al color del bloc
        if color_id == RD:
            eyes_fill(RED)
            time.sleep(1.0)
            _execute_action(task_move_to, CROSSING)

        elif color_id == GN:
            eyes_fill(GREEN)
            time.sleep(1.0)
            _execute_action(task_rotate, CW)

        elif color_id == BU:
            eyes_fill(BLUE)
            time.sleep(1.0)
            _execute_action(task_rotate, CCW)

        elif color_id == YE:
            eyes_fill(YELLOW)
            time.sleep(1.0)
            _execute_action(task_take_or_leave_something, TAKE)

        elif color_id == OG:
            eyes_fill(ORANGE)
            time.sleep(1.0)
            _execute_action(task_take_or_leave_something, LEAVE)

        elif color_id == VT:
            eyes_fill(PURPLE)
            time.sleep(1.0)
            _execute_action(task_move_backward)

        eyes_idle()

        # Expulsa el bloc
        servo_move_to(EJECT_POSITION)
        time.sleep(EJECT_BLOCK_MS)


# ==================
# Tasca de gestos
# ==================

def _wait_gesture_confirmation() -> bool:
    """
    Espera un gest de confirmació mentre els ulls parpellegen la comanda.
    AMUNT = confirmar (True), AVALL = cancel·lar (False), timeout = cancel·lar.
    """
    start = time.time()
    while (time.time() - start < CONFIRM_TIMEOUT_S
           and not _shutdown_event.is_set()):
        g = read_gesture()
        if g == GS_UP:
            return True
        if g == GS_DOWN:
            return False
        time.sleep(0.05)
    return False   # timeout -> cancel·la


def task_read_gestures():
    """
    Mode gestos amb CONFIRMACIÓ:
      1) Es llegeix un gest de comanda -> els ulls PARPELLEGEN del color de la
         comanda (esperant confirmació).
      2) Gest AMUNT = confirmar (tick verd) -> executa.
         Gest AVALL = cancel·lar (creu vermella).
         Sense resposta en CONFIRM_TIMEOUT_S -> cancel·la.
      3) Després: FLUSH del buffer + COOLDOWN (no memoritza gestos fets durant
         l'acció ni encadena rebots).
    GS_WAVE (fora de confirmació) commuta mode blocs <-> gestos.

    Mapatge NATURAL (la correcció d'orientació del sensor girat es fa TOTA al
    driver gesture.py, que ja reporta el gest físic):
      UP->endavant  DOWN->enrere  LEFT->girar esquerra  RIGHT->girar dreta
      CW->buidar    CCW->xuclar
    """
    global _gesture_mode_active

    # gest de comanda -> (color dels ulls, (funció d'acció, *args))
    commands = {
        GS_UP:            (RED,    (task_move_to, CROSSING)),      # endavant
        GS_DOWN:          (PURPLE, (task_move_backward,)),         # enrere
        # ⚠️ Esquerra i dreta van INVERTIDES respecte del bloc del mateix color,
        # i és intencionat: qui fa gestos està DAVANT del robot, mirant-lo a la
        # cara, o sigui que la seva esquerra és la dreta del robot. Fer-ho com
        # un mirall és el que resulta natural per a qui el controla.
        GS_LEFT:          (GREEN,  (task_rotate, CW)),             # la seva esquerra
        GS_RIGHT:         (BLUE,   (task_rotate, CCW)),            # la seva dreta
        GS_CLOCKWISE:     (ORANGE, (task_take_or_leave_something, LEAVE)),  # buidar
        GS_ANTICLOCKWISE: (YELLOW, (task_take_or_leave_something, TAKE)),   # xuclar
    }

    while not _shutdown_event.is_set():
        # Cedir CPU SEMPRE, i molt més mentre el robot es mou. Abans, quan el
        # sensor no reportava cap gest, es tornava a llegir immediatament:
        # milers de lectures I²C per segon competint amb el generador de
        # polsos, que és el que limita la velocitat del robot.
        time.sleep(GESTURE_POLL_MOVING_S if motion.moving else GESTURE_POLL_S)

        gesture_id = read_gesture()

        # WAVE: toggle entre mode blocs i mode gestos (sense confirmació)
        if gesture_id == GS_WAVE:
            with _mode_lock:
                _gesture_mode_active = not _gesture_mode_active
                active = _gesture_mode_active
            print("Gesture mode ON" if active else "Gesture mode OFF")
            eyes_mode_switch(active)    # senyal visual del mode nou
            eyes_idle()
            time.sleep(0.4)
            read_gesture()          # flush
            continue

        with _mode_lock:
            active = _gesture_mode_active
        if not active or gesture_id not in commands:
            continue

        # --- Comanda llegida: fase de CONFIRMACIÓ ---
        color, action = commands[gesture_id]
        eyes_blink_color(color)     # parpelleja el color: esperant confirmació
        time.sleep(0.3)
        read_gesture()              # flush del gest de comanda

        if _wait_gesture_confirmation():
            eyes_tick()             # tick verd: confirmat
            time.sleep(CONFIRM_FEEDBACK_S)
            eyes_fill(color)        # color de la comanda mentre s'executa
            _execute_action(*action)
        else:
            eyes_cross()            # creu vermella: cancel·lat
            time.sleep(CONFIRM_FEEDBACK_S)

        eyes_idle()
        read_gesture()              # flush gestos fets durant l'acció (sense memòria)
        time.sleep(GESTURE_COOLDOWN_S)


# ==================
# Shutdown
# ==================

def _ip_local() -> str:
    """IP d'aquesta màquina a la xarxa, per dir per on s'hi accedeix."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))       # no envia res, només tria la ruta
        return s.getsockname()[0]
    except OSError:
        return "<ip-del-robot>"
    finally:
        s.close()


def _shutdown(sig, frame):
    print("\nAturant QuiBot...")
    _shutdown_event.set()


# ==================
# Main
# ==================

def main():
    global _pi

    # Connecta amb pigpiod (ha d'estar en marxa amb: sudo pigpiod -s 1)
    _pi = pigpio.pi()
    if not _pi.connected:
        print("ERROR: No s'ha pogut connectar a pigpiod. Executa: sudo pigpiod -s 1")
        sys.exit(1)

    # ULLS PRIMER: aixi mostrem un estat "arrencant" mentre es prepara la resta.
    # (Els LEDs es poden encendre abans que el robot estigui llest per a accions.)
    eyes_setup(_pi)
    eyes_blink()                # estat NO-PREPARAT / arrencant (ratlla blanca)

    # Inicialitza la resta de mòduls
    blocks_setup(_pi)
    motion_setup(_pi)
    gesture_setup()

    # Ni els braços ni la xeringa fan homing a l'arrencada: tots dos parteixen
    # sempre de zero. Els braços perquè, sense molles, cauen al topall en
    # apagar-se, i a més motion_cleanup() els hi baixa controladament abans;
    # la xeringa perquè acaba cada cicle a zero i el pistó no rellisca amb els
    # motors apagats. Totes dues funcions es conserven per poder recalibrar
    # sota demanda des de la interfície web.
    #   arms_home()
    #   syringe_home()
    enable_arms(ON)
    move_arms_up()      # els braços no poden tapar el sensor de gestos

    # Els ulls continuen amb l'animació de càrrega: encara falten els threads
    # i el servidor web. El senyal de "llest" es dona al final, quan de veritat
    # es pot controlar el robot.

    # Registra els senyals de sortida
    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Arrenca els threads principals
    t_blocks   = threading.Thread(target=task_read_blocks,   daemon=True, name="blocks")
    t_gestures = threading.Thread(target=task_read_gestures, daemon=True, name="gestures")

    t_blocks.start()
    t_gestures.start()

    # Servidor web: comparteix procés amb els altres dos modes, de manera que
    # tots tres veuen el mateix estat i el mateix maquinari.
    try:
        import webserver
        webserver.bind(_RobotBridge())
        threading.Thread(target=webserver.run, daemon=True, name="web").start()
        print(f"Control remot a http://{_ip_local()}:{webserver.PORT}")
    except Exception as e:                                  # noqa: BLE001
        # Que el robot funcioni encara que la web falli: gestos i blocs no
        # depenen d'ella.
        print(f"AVÍS: el servidor web no ha arrencat ({e})")

    eyes_idle()                 # ARA sí: tot en marxa, es pot controlar
    print("QuiBot llest.")

    # Espera senyal de sortida
    _shutdown_event.wait()

    # Cleanup ordenat. L'animació d'apagada va PRIMER: motion_cleanup() triga
    # uns segons baixant els braços, i durant aquella estona els ulls es
    # quedaven amb l'últim estat, com si el robot encara estigués actiu.
    eyes_blink()          # mateixa animació que a l'arrencada: "treballant"
    motion_cleanup()      # triga uns segons: baixa els braços
    eyes_cleanup()        # i ara sí, apagats (atura el fil de render i tanca)
    gesture_cleanup()
    _pi.stop()
    print("QuiBot aturat.")


if __name__ == "__main__":
    main()
