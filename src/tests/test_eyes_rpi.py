"""
test_eyes_rpi.py — Test autonom dels ulls WS2811 amb rpi_ws281x (GPIO13, PWM1).

Verifica que, movent la linia de dades a GPIO13 (pont fisic des de GPIO9) i
usant rpi_ws281x (timing per HARDWARE), els colors surten CORRECTES — a
diferencia de pigpio, que feia sortir tot blanc (el bit "0" es llegia com "1").

Cicla: VERMELL, VERD, BLAU, BLANC, APAGAT (tots 128 LEDs).
  - Es veuen els colors correctes  -> timing OK, migracio reeixida. :)
  - Es veu tot blanc                -> encara timing dolent (revisar pin/canal).
  - Colors intercanviats (R<->G)    -> canviar strip_type (GRB<->RGB) mes avall.

REQUISITS:
  - Pont fisic GPIO13 -> GPIO9 fet, i GPIO9 en INPUT (aquest test l'hi posa).
  - Llibreria: sudo pip install rpi_ws281x   (o dins el venv)
  - EXECUTAR COM A ROOT (rpi_ws281x per PWM ho necessita):
        sudo ~/.venv/bin/python3 tests/test_eyes_rpi.py
    (ajusta la ruta del python del venv si cal)
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pins import LED_DATA, LED_DATA_OLD

try:
    from rpi_ws281x import PixelStrip, Color, ws
except ImportError:
    print("ERROR: falta la llibreria rpi_ws281x.")
    print("Instal·la-la:  sudo pip install rpi_ws281x")
    sys.exit(1)

# --- Configuracio de la tira ---
LED_COUNT      = 128          # 2 matrius 8x8
LED_PIN        = LED_DATA     # GPIO13 (PWM1)
LED_FREQ_HZ    = 800000       # 800 kHz (WS2811 mode rapid); si falla, prova 400000
LED_DMA        = 10           # canal DMA
LED_INVERT     = False
LED_BRIGHTNESS = 128          # 0-255 (comença a mitja brillantor, per no enlluernar)
LED_CHANNEL    = 1            # GPIO13 = PWM1 -> canal 1  (GPIO12/18 serien canal 0)
LED_STRIP      = ws.WS2811_STRIP_GRB   # ordre GRB (com FastLED); si R/G surten
                                       # canviats, prova ws.WS2811_STRIP_RGB


def _gpio9_input():
    """Deixa GPIO9 (pin antic) en alta impedancia perque no lluiti amb GPIO13."""
    try:
        import pigpio
        pi = pigpio.pi()
        if pi.connected:
            pi.set_mode(LED_DATA_OLD, pigpio.INPUT)
            pi.stop()
            print(f"GPIO{LED_DATA_OLD} posat en INPUT (via pigpio).")
            return
    except Exception:
        pass
    print(f"AVIS: no s'ha pogut forçar GPIO{LED_DATA_OLD} a INPUT per software.")
    print("      (per defecte arrenca com a input; si ve d'un test anterior que")
    print("       el deixava en output, reinicia la Pi abans d'aquest test.)")


def fill(strip, color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()


def main():
    _gpio9_input()

    strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
                       LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
    strip.begin()
    print(f"Tira iniciada: {LED_COUNT} LEDs a GPIO{LED_PIN} (canal {LED_CHANNEL}).")

    seq = [
        ("VERMELL", Color(255, 0, 0)),
        ("VERD",    Color(0, 255, 0)),
        ("BLAU",    Color(0, 0, 255)),
        ("BLANC",   Color(255, 255, 255)),
        ("APAGAT",  Color(0, 0, 0)),
    ]
    try:
        for nom, color in seq:
            print(f"  -> {nom}  (que veus realment?)")
            fill(strip, color)
            time.sleep(2.5)
    finally:
        fill(strip, Color(0, 0, 0))
        print("Fi.")


if __name__ == "__main__":
    main()
