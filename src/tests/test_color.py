"""
test_color.py — Verificacio i calibracio del sensor de color TCS34725.

Dues modalitats:
  1) Lectura en viu — mostra RGB cru, les fraccions normalitzades i la
     classificacio (classify_color de blocks.py) en bucle. Serveix per
     comprovar que el sensor identifica be cada bloc.
  2) Captura de calibracio — per a cada color (i sense bloc), pren 30 lectures
     mentre mous lleugerament el bloc, i informa la MITJANA i el RANG (min-max)
     de R,G,B i de les fraccions %. Dona les dades per recalibrar la taula
     _COLORS de blocks.py i veure quins colors se solapen.

La classificacio es per HUE normalitzat (independent de la brillantor), igual
que al robot real. Cal recalibrar sempre que canviï la il.luminacio de la sala.

Sensor al bus I2C 1 (hardware GPIO2/3, 0x29). NO cal pigpiod.
Executa des de Rasp/:   python3 tests/test_color.py
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import adafruit_extended_bus
import adafruit_tcs34725
from blocks import _COLORS, classify_color, MIN_TOTAL

COLORS_CALIB = [
    "SENSE BLOC (ambient)",
    "VERMELL (RD)",
    "VERD (GN)",
    "BLAU (BU)",
    "GROC (YE)",
    "TARONJA (OG)",
    "VIOLETA (VT)",
]
N = 30   # lectures per color a la calibracio


def open_sensor():
    """Inicialitza el TCS34725 al bus I2C 1. Surt del programa si falla."""
    try:
        i2c = adafruit_extended_bus.ExtendedI2C(1)   # hardware GPIO2/3
        sensor = adafruit_tcs34725.TCS34725(i2c)
        sensor.integration_time = 100  # ms (pujat de 50: mes senyal, menys soroll)
        sensor.gain = 4                # 4x
        return sensor
    except Exception as e:
        print(f"ERROR inicialitzant el TCS34725 al bus 1: {e}")
        print("Comprova amb 'i2cdetect -y 1' que hi surt el 0x29.")
        sys.exit(1)


# ------------------------------------------------------------------
# 1) Lectura en viu
# ------------------------------------------------------------------

def best_diff(r, g, b):
    """Diferencia normalitzada (x1000) al color de referencia mes proper."""
    total = r + g + b
    if total < MIN_TOTAL:
        return 0.0
    nr, ng, nb = r / total, g / total, b / total
    bd = 99999.0
    for ref in _COLORS:
        d = (abs(nr - ref["nr"]) + abs(ng - ref["ng"]) + abs(nb - ref["nb"])) * 1000
        bd = min(bd, d)
    return bd


def live_read(sensor):
    print("\n=== LECTURA EN VIU (hue normalitzat) ===")
    print("Posa blocs davant del sensor. Ctrl+C per tornar al menu.\n")
    print(f"  {'R':>3} {'G':>3} {'B':>3}  | {'%R':>3} {'%G':>3} {'%B':>3} |  color  diff")
    try:
        while True:
            r, g, b = sensor.color_rgb_bytes
            total = r + g + b
            if total > 0:
                pr, pg, pb = 100 * r / total, 100 * g / total, 100 * b / total
            else:
                pr = pg = pb = 0
            cid = classify_color(r, g, b)
            name = _COLORS[cid]["name"]
            diff = best_diff(r, g, b)
            print(f"  {r:3d} {g:3d} {b:3d}  | {pr:3.0f} {pg:3.0f} {pb:3.0f} |  {name:2s}   {diff:4.0f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n(tornant al menu)")


# ------------------------------------------------------------------
# 2) Captura de calibracio
# ------------------------------------------------------------------

def _stat(xs):
    return (sum(xs) / len(xs), min(xs), max(xs))


def capture(sensor, nom):
    input(f"\n>> Posa: {nom}   i prem Enter (mou-lo una mica durant la captura)...")
    rs, gs, bs = [], [], []
    prs, pgs, pbs = [], [], []
    for _ in range(N):
        r, g, b = sensor.color_rgb_bytes
        t = r + g + b
        rs.append(r); gs.append(g); bs.append(b)
        if t > 0:
            prs.append(100 * r / t); pgs.append(100 * g / t); pbs.append(100 * b / t)
        else:
            prs.append(0); pgs.append(0); pbs.append(0)
        time.sleep(0.05)

    ar, mnr, mxr = _stat(rs)
    ag, mng, mxg = _stat(gs)
    ab, mnb, mxb = _stat(bs)
    par = sum(prs) / len(prs)
    pag = sum(pgs) / len(pgs)
    pab = sum(pbs) / len(pbs)

    print(f"   RGB mitjana = ({ar:5.1f}, {ag:5.1f}, {ab:5.1f})"
          f"   rang R[{mnr}-{mxr}] G[{mng}-{mxg}] B[{mnb}-{mxb}]")
    print(f"   %   mitjana = ({par:4.0f}%, {pag:4.0f}%, {pab:4.0f}%)")


def calibrate(sensor):
    print("\n=== CAPTURA DE CALIBRACIO ===")
    for nom in COLORS_CALIB:
        capture(sensor, nom)
    print("\nFet. Les mitjanes i els rangs son el que cal portar a _COLORS de blocks.py.")


# ------------------------------------------------------------------

def main():
    print("=== SENSOR DE COLOR TCS34725 ===")
    sensor = open_sensor()
    print("Sensor OK.")
    while True:
        print("\n  1) Lectura en viu i classificacio")
        print("  2) Captura de calibracio (30 lectures per color)")
        print("  0) Sortir")
        try:
            opt = input("Opcio: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if opt == "1":
            live_read(sensor)
        elif opt == "2":
            calibrate(sensor)
        elif opt == "0" or not opt:
            break
    print("Fi.")


if __name__ == "__main__":
    main()
