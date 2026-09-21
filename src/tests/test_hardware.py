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
test_hardware.py — Test interactiu del hardware del Qui-Bot H2O.

Prova cada element per separat per verificar el muntatge i calibrar els sensors:
  - Els 5 motors pas a pas, individualment i en parelles (rodes / braços)
  - Els 3 finals de carrera d'efecte Hall (lectura en viu)
  - El sensor de distància VL53L0X (lectura en viu, per calibrar)
  - Els sensors de línia TCRT5000 via ADS1115 (lectura en viu, per calibrar)
  - El servomotor d'expulsió de blocs (posicions, cicles i escombrat)

Ús (a la Raspberry, dins de Rasp/):
    sudo ~/.venv/bin/python3 tests/test_hardware.py

Requisits:
    - pigpiod en marxa amb resolució 1us:  sudo pigpiod -s 1 -x 0xFFFFFFFF
    - Per als sensors I2C, /boot/firmware/config.txt ha de tenir els busos:
        dtparam=i2c_arm=on                      # bus hardware GPIO2/3 -> /dev/i2c-1
        dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1   # GPIO0/1 -> /dev/i2c-3
      i el modul:  sudo modprobe i2c-dev

NOTA PCB: en aquesta placa l'ENABLE del driver de la xeringa esta cablejat a
EN_A (el dels bracos), no a EN_SERVO. Per aixo, per moure la xeringa, aquest
test activa EN_A (i tambe EN_SERVO, per cobrir tots dos casos).
"""

import sys
import time
import pigpio

# Permet executar des de Rasp/ o des de Rasp/tests/
sys.path.insert(0, "..")
sys.path.insert(0, ".")

import motion
from motion import (
    motion_setup_steppers,
    enable_wheels, enable_arms, enable_syringe,
    is_endstop_detecting,
    ON, OFF,
)
import pins

# ---- Busos I2C dels sensors (ajusta'ls si el teu config.txt es diferent) ----
BUS_ADS  = 1   # ADS1115 (sensors de linia) -> bus hardware GPIO2/3 (/dev/i2c-1)
BUS_DIST = 3   # VL53L0X (distancia) -> bus software GPIO0/1 (/dev/i2c-3)

STEP_TEST = 400   # passos de prova per a cada motor


# ==========================================================================
# Utilitats
# ==========================================================================

def _connect():
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpiod no esta en marxa. Executa: sudo pigpiod -s 1 -x 0xFFFFFFFF")
        sys.exit(1)
    return pi


def _wait_until_stopped(stepper, timeout=15.0):
    """Espera que un stepper arribi a la posicio objectiu (amb timeout de seguretat)."""
    t0 = time.time()
    while stepper.distance_to_go() != 0:
        if time.time() - t0 > timeout:
            print("  AVIS: timeout esperant el motor (no s'ha mogut prou rapid?)")
            stepper.stop()
            return False
        time.sleep(0.05)
    return True


def _move_motor(stepper, enable_fn, steps, label):
    """Activa el driver, mou el motor 'steps' (signat), espera i desactiva."""
    print(f"  -> {label}: movent {steps:+d} passos...")
    enable_fn(ON)
    time.sleep(0.1)
    start = stepper.current_position()
    stepper.move(steps)
    _wait_until_stopped(stepper)
    moved = stepper.current_position() - start
    enable_fn(OFF)
    print(f"     fet (posicio {start} -> {stepper.current_position()}).")
    return moved


def _enable_syringe_real(state):
    """La xeringa s'habilita per EN_A en aquesta PCB; activem EN_A i EN_SERVO."""
    enable_arms(state)
    enable_syringe(state)


# ==========================================================================
# Tests de motors
# ==========================================================================

def _motor_table():
    """
    Llista (nom, stepper, funcio_enable, sentit+, sentit-) dels 5 motors.
    Cridar despres del setup.

    Convencio de motion.py: positiu = ENDAVANT a les rodes i AMUNT als bracos.
    Les etiquetes diuen que ha de passar fisicament, aixi el test serveix per
    verificar el sentit i no nomes que el motor es mogui.
    """
    return [
        ("RODA dreta",     motion.wheel_R, enable_wheels,        "endavant", "enrere"),
        ("RODA esquerra",  motion.wheel_L, enable_wheels,        "endavant", "enrere"),
        ("BRAC dret",      motion.arm_R,   enable_arms,          "amunt",    "avall"),
        ("BRAC esquerre",  motion.arm_L,   enable_arms,          "amunt",    "avall"),
        ("XERINGA",        motion.syringe, _enable_syringe_real, "surt",     "entra"),
    ]


def test_motors_individual(pi):
    """Prova els 5 motors un per un: endavant i enrere, amb confirmacio."""
    print("\n=== TEST MOTORS INDIVIDUALS ===")
    print("Es provara cada motor per separat. Mira que cadascun es mou.\n")
    for name, stepper, en, pos, neg in _motor_table():
        input(f"[Enter] per provar el motor: {name} (Ctrl+C per saltar)")
        try:
            _move_motor(stepper, en, +STEP_TEST, f"{name}: ha d'anar {pos.upper()}")
            time.sleep(0.5)
            _move_motor(stepper, en, -STEP_TEST, f"{name}: ha d'anar {neg.upper()}")
        except KeyboardInterrupt:
            print("  (saltat)")
        r = input(f"  {name}: s'ha mogut en el sentit correcte? (s/n): ").strip().lower()
        print(f"  -> {name}: {'OK' if r == 's' else 'REVISAR'}\n")
    print("Test de motors individuals completat.\n")


def test_wheels_pair(pi):
    """Prova les dues rodes alhora: mateix sentit (avancar) i sentits oposats (girar)."""
    print("\n=== TEST PARELLA DE RODES ===")
    input("[Enter] per fer avancar les dues rodes (mateix sentit)...")
    enable_wheels(ON); time.sleep(0.1)
    motion.wheel_R.move(+STEP_TEST); motion.wheel_L.move(+STEP_TEST)
    _wait_until_stopped(motion.wheel_R); _wait_until_stopped(motion.wheel_L)
    enable_wheels(OFF)
    print("  Avancar: fet.")

    input("[Enter] per fer girar (rodes en sentits oposats)...")
    enable_wheels(ON); time.sleep(0.1)
    motion.wheel_R.move(+STEP_TEST); motion.wheel_L.move(-STEP_TEST)
    _wait_until_stopped(motion.wheel_R); _wait_until_stopped(motion.wheel_L)
    enable_wheels(OFF)
    print("  Girar: fet.\n")


def test_arms_pair(pi):
    """Prova els dos bracos alhora: han de pujar i baixar TOTS DOS a l'hora."""
    print("\n=== TEST PARELLA DE BRACOS ===")
    print("Els dos bracos han de pujar alhora i despres baixar alhora.")
    print("Si un puja i l'altre baixa, un dels dos te el sentit invertit.")
    input("[Enter] per moure els dos bracos AMUNT i despres AVALL...")
    enable_arms(ON); time.sleep(0.1)
    motion.arm_R.move(+STEP_TEST); motion.arm_L.move(+STEP_TEST)
    _wait_until_stopped(motion.arm_R); _wait_until_stopped(motion.arm_L)
    time.sleep(0.5)
    motion.arm_R.move(-STEP_TEST); motion.arm_L.move(-STEP_TEST)
    _wait_until_stopped(motion.arm_R); _wait_until_stopped(motion.arm_L)
    enable_arms(OFF)
    print("  Bracos: fet.\n")


def test_endstops(pi):
    """Llegeix en viu els 3 finals de carrera Hall. Acosta un iman per provar-los."""
    print("\n=== TEST FINALS DE CARRERA (Hall) ===")
    print("Acosta un iman a cada sensor. 1 = detecta, 0 = lliure. Ctrl+C per sortir.\n")
    try:
        while True:
            la = is_endstop_detecting(pins.END_LA)
            ra = is_endstop_detecting(pins.END_RA)
            sy = is_endstop_detecting(pins.END_SY)
            print(f"  brac_esq(END_LA)={int(la)}  brac_dret(END_RA)={int(ra)}  xeringa(END_SY)={int(sy)}   ", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n  (fi)\n")


# ==========================================================================
# Tests de sensors (per calibrar)
# ==========================================================================

def test_distance(pi):
    """Lectura en viu del VL53L0X (mm). Posa la ma davant per veure variar."""
    print("\n=== TEST SENSOR DISTANCIA (VL53L0X) ===")
    try:
        import adafruit_extended_bus
        import adafruit_vl53l0x
        i2c = adafruit_extended_bus.ExtendedI2C(BUS_DIST)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
    except Exception as e:
        print(f"  ERROR inicialitzant el VL53L0X al bus {BUS_DIST}: {e}")
        print(f"  Comprova el bus (config.txt) i la connexio. Bus actual BUS_DIST={BUS_DIST}.")
        return
    print("Llegint distancia. Mou la ma davant del sensor. Ctrl+C per sortir.\n")
    try:
        while True:
            try:
                d = sensor.range
                status = getattr(sensor, "range_status", 0)
                if status != 0:
                    print("  distancia = (sense objectiu / fora de rang)  ", end="\r")
                else:
                    print(f"  distancia = {d:5d} mm                          ", end="\r")
            except Exception:
                print("  distancia = (error de lectura)               ", end="\r")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n  (fi)\n")


def test_line_sensors(pi):
    """
    Lectura en viu dels sensors de linia (TCRT5000 via ADS1115).
    Per calibrar BLACK_THRESHOLD: anota els valors sobre BLANC i sobre NEGRE.
    """
    print("\n=== TEST SENSORS DE LINIA (TCRT5000 / ADS1115) ===")
    try:
        import adafruit_extended_bus
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn
        i2c = adafruit_extended_bus.ExtendedI2C(BUS_ADS)
        ads = ADS.ADS1115(i2c)
        ch_r = AnalogIn(ads, 0)   # LINES_R = canal 0 (AIN0)
        ch_l = AnalogIn(ads, 1)   # LINES_L = canal 1 (AIN1)
    except Exception as e:
        print(f"  ERROR inicialitzant l'ADS1115 al bus {BUS_ADS}: {e}")
        print(f"  Comprova el bus (config.txt) i la connexio. Bus actual BUS_ADS={BUS_ADS}.")
        return
    # Els LED IR dels TCRT5000 es commuten per Q2 (NPN) amb la base a EN_W:
    # s'encenen quan EN_W esta a HIGH = rodes DESHABILITADES => enable_wheels(OFF).
    # (Polaritat real de la PCB, inversa de l'esperat.)
    print("Alimentant els LED IR (EN_W a HIGH, rodes deshabilitades)...")
    enable_wheels(OFF)
    time.sleep(0.3)
    print("\nPosa els sensors sobre BLANC i sobre NEGRE i anota els valors.")
    print(f"Llindar actual BLACK_THRESHOLD = {motion.BLACK_THRESHOLD}. Ctrl+C per sortir.\n")
    try:
        while True:
            r, l = ch_r.value, ch_l.value
            print(f"  dret={r:6d}  esquerre={l:6d}  (diff={r-l:+6d})     ", end="\r")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n  (fi)\n")
    finally:
        enable_wheels(OFF)


# ==========================================================================
# Menu principal
# ==========================================================================

# ==========================================================================
# Servomotor d'expulsio de blocs
# ==========================================================================

# Valors del servo (han de coincidir amb blocks.py)
MIN_SERVO_US   = 1000   # ~0 graus
MAX_SERVO_US   = 2440   # ~180 graus
OPEN_POSITION  = 2050   # posicio oberta (repos) — calibrat 2026-07 al robot
EJECT_POSITION = 1550   # posicio d'expulsio del bloc — calibrat 2026-07
INCREMENT_US   = 3      # us per pas (moviment suau)

_servo_current = OPEN_POSITION


def _servo_move_to(pi, target):
    """Mou el servo suaument fins a 'target' us (limitat al rang)."""
    global _servo_current
    target = max(MIN_SERVO_US, min(MAX_SERVO_US, int(target)))
    while _servo_current != target:
        if _servo_current < target - INCREMENT_US:
            _servo_current += INCREMENT_US
        elif _servo_current > target + INCREMENT_US:
            _servo_current -= INCREMENT_US
        else:
            _servo_current = target
        pi.set_servo_pulsewidth(pins.SERVO_PWM, _servo_current)
        time.sleep(0.001)


SERVO_MENU = """
  --- SERVO ---
    1) OPEN   (obert / repos)
    2) EJECT  (expulsar bloc)
    3) Cicle OPEN -> EJECT -> OPEN  (x3)
    4) Posicio concreta en us  (per calibrar)
    5) Escombrat MIN <-> MAX
    0) Tornar al menu principal"""


def test_servo(pi):
    """Test interactiu del servo d'expulsio de blocs."""
    global _servo_current
    # Q1 esta PONTEJAT (C-E) -> la massa del servo va DIRECTA a GND sempre, no
    # depen de EN_SERVO. Deixem EN_SERVO a LOW per no injectar corrent parasit
    # per R3 cap a la base. El servo ja te massa permanent gracies al pont.
    pi.set_mode(pins.EN_SERVO, pigpio.OUTPUT)
    pi.write(pins.EN_SERVO, 0)

    pi.set_mode(pins.SERVO_PWM, pigpio.OUTPUT)
    _servo_current = OPEN_POSITION
    pi.set_servo_pulsewidth(pins.SERVO_PWM, _servo_current)
    time.sleep(0.5)
    print(f"\nServo a GPIO{pins.SERVO_PWM} (Q1 pontejat -> massa directa).  "
          f"OPEN={OPEN_POSITION}  EJECT={EJECT_POSITION}  "
          f"(rang {MIN_SERVO_US}-{MAX_SERVO_US} us)")
    try:
        while True:
            print(SERVO_MENU)
            opt = input("  Opcio: ").strip().lower()
            if opt == "1":
                _servo_move_to(pi, OPEN_POSITION); print("    -> OPEN")
            elif opt == "2":
                _servo_move_to(pi, EJECT_POSITION); print("    -> EJECT")
            elif opt == "3":
                for _ in range(3):
                    _servo_move_to(pi, EJECT_POSITION); time.sleep(0.4)
                    _servo_move_to(pi, OPEN_POSITION);  time.sleep(0.4)
                print("    -> cicle fet")
            elif opt == "4":
                try:
                    us = int(input(f"    us ({MIN_SERVO_US}-{MAX_SERVO_US}): "))
                    _servo_move_to(pi, us)
                    print(f"    -> {max(MIN_SERVO_US, min(MAX_SERVO_US, us))} us")
                except ValueError:
                    print("    valor no valid")
            elif opt == "5":
                _servo_move_to(pi, MIN_SERVO_US); time.sleep(0.5)
                _servo_move_to(pi, MAX_SERVO_US); time.sleep(0.5)
                _servo_move_to(pi, OPEN_POSITION)
                print("    -> escombrat fet")
            elif opt == "0" or not opt:
                break
            else:
                print("    opcio no valida")
    finally:
        pi.set_servo_pulsewidth(pins.SERVO_PWM, 0)   # atura el PWM (allibera el servo)


MENU = """
========= TEST HARDWARE QUI-BOT H2O =========
  MOTORS
    1) Motors individuals (els 5, un per un)
    2) Parella de rodes (avancar / girar)
    3) Parella de bracos
    4) Finals de carrera Hall (lectura en viu)
  SENSORS (calibracio)
    5) Sensor de distancia VL53L0X
    6) Sensors de linia ADS1115
  ACTUADORS
    7) Servomotor d'expulsio de blocs
    0) Sortir
=============================================="""


def main():
    pi = _connect()
    print("Inicialitzant motors (motion_setup_steppers)...")
    motion_setup_steppers(pi)
    tests = {
        "1": test_motors_individual, "2": test_wheels_pair, "3": test_arms_pair,
        "4": test_endstops, "5": test_distance, "6": test_line_sensors,
        "7": test_servo,
    }
    try:
        while True:
            print(MENU)
            opt = input("Opcio: ").strip()
            if opt == "0":
                break
            fn = tests.get(opt)
            if fn:
                try:
                    fn(pi)
                except KeyboardInterrupt:
                    print("\n  (test interromput)\n")
            else:
                print("Opcio no valida.")
    finally:
        print("Aturant i desactivant motors...")
        try:
            enable_wheels(OFF); enable_arms(OFF); enable_syringe(OFF)
        except Exception:
            pass
        motion._stepper_stop.set()
        time.sleep(0.2)
        pi.stop()
        print("Fet.")


if __name__ == "__main__":
    main()
