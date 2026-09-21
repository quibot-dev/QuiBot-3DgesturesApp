"""
test_gesture.py — Verificació del sensor de gestos PAJ7620U2.

Dues modalitats:
  1) Comprovació de connexió — inicialitza el sensor i comprova que respon
     per I2C. Serveix per descartar un problema de cablatge o d'adreça.
  2) Lectura en viu — mostra cada gest detectat. Serveix per verificar que
     el sensor distingeix bé les vuit direccions i a quina distància.

El sensor és al bus I2C 1 (hardware GPIO2/3, adreça 0x73), compartit amb el
sensor de color i l'ADS1115. Fa servir smbus2 directament: NO cal pigpiod.

Executa des de Rasp/:   python3 tests/test_gesture.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from gesture import (
    gesture_setup, gesture_cleanup,
    read_gesture,
    GS_NONE, GS_FORWARD, GS_BACKWARD, GS_LEFT, GS_RIGHT,
    GS_UP, GS_DOWN, GS_CLOCKWISE, GS_ANTICLOCKWISE, GS_WAVE,
)

_GESTURE_NAMES = {
    GS_NONE:          "Cap (GS_NONE)",
    GS_FORWARD:       "Endavant (GS_FORWARD)",
    GS_BACKWARD:      "Enrere (GS_BACKWARD)",
    GS_LEFT:          "Esquerra (GS_LEFT)",
    GS_RIGHT:         "Dreta (GS_RIGHT)",
    GS_UP:            "Amunt (GS_UP)",
    GS_DOWN:          "Avall (GS_DOWN)",
    GS_CLOCKWISE:     "Horari (GS_CLOCKWISE)",
    GS_ANTICLOCKWISE: "Antihorari (GS_ANTICLOCKWISE)",
    GS_WAVE:          "Wave (GS_WAVE)",
}


# ==================
# TEST 1 — Connexió i inicialització
# ==================

def test_connection():
    """
    Comprova que el sensor PAJ7620U2 és accessible via I2C.
    Ha de mostrar 'Gesture sensor init OK' sense errors.
    """
    print("=== TEST CONNEXIÓ PAJ7620U2 ===")

    gesture_setup()
    time.sleep(0.5)

    gesture_cleanup()
    print("Test connexió completat.\n")


# ==================
# TEST 2 — Lectura de gestos
# ==================

def test_read_gestures():
    """
    Mostra cada gest detectat fins que premis Ctrl+C.
    Fes gestos davant del sensor per verificar que els detecta correctament:
      - Mà cap endavant/enrere  → GS_FORWARD
      - Mà cap a l'esquerra     → GS_LEFT
      - Mà cap a la dreta       → GS_RIGHT
      - Mà cap amunt            → GS_UP
      - Mà cap avall            → GS_DOWN
      - Rotació horària         → GS_CLOCKWISE
      - Rotació antihorària     → GS_ANTICLOCKWISE
      - Sacsejada (wave)        → GS_WAVE
    """
    print("=== TEST LECTURA GESTOS PAJ7620U2 ===")
    print("Fes gestos davant del sensor (Ctrl+C per tornar al menu)...")

    gesture_setup()
    time.sleep(0.5)

    try:
        while True:
            gest = read_gesture()
            if gest != GS_NONE:
                nom = _GESTURE_NAMES.get(gest, f"Desconegut ({gest})")
                print(f"  Gest detectat: {nom}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_cleanup()
    print("\nTest lectura gestos completat.\n")


# ==================
# Execució
# ==================

def main():
    print("=== SENSOR DE GESTOS PAJ7620U2 ===")
    while True:
        print("\n  1) Comprovacio de connexio")
        print("  2) Lectura de gestos en viu")
        print("  0) Sortir")
        try:
            opt = input("Opcio: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if opt == "1":
            test_connection()
        elif opt == "2":
            test_read_gestures()
        elif opt == "0" or not opt:
            break
        else:
            print("  opcio no valida")
    print("Fi.")


if __name__ == "__main__":
    main()
