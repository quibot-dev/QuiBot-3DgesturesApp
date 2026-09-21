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
pins.py — Definició de pins GPIO (BCM) de la Raspberry Pi Zero 2W.

Valors extrets del connector RASP1 del projecte KiCad de la placa
(projecte-quibot-master), verificats pin a pin contra l'esquemàtic.

Tots els números fan referència a la numeració BCM.
"""

# ==================
# MOTORS (drivers A4988)
# ==================

# Enables (actius a LOW), compartits per parelles
EN_W     = 4    # enable rodes
EN_A     = 5    # enable braços
EN_SERVO = 6    # enable subsistema servo + xeringa

# Servo d'expulsió de blocs
SERVO_PWM = 8   # GPIO8 (CE0)

# Motor pas a pas xeringa
STEP_SY = 17
DIR_SY  = 7

# Motor pas a pas roda dreta
STEP_R_W = 27
DIR_R_W  = 16

# Motor pas a pas roda esquerra
STEP_L_W = 14
DIR_L_W  = 22

# Motor pas a pas braç dret
STEP_R_A = 25
DIR_R_A  = 26

# Motor pas a pas braç esquerre
STEP_L_A = 23
DIR_L_A  = 24

# ==================
# SENSORS — busos I2C
# ==================

# Bus I2C del sensor de color TCS34725 (GPIO2/3, hardware i2c-1).
# En aquest MATEIX bus hi pengen també el PAJ7620U2 (gestos) i l'ADS1115 (línia).
SDA_COL = 2
SCL_COL = 3

# Sensor de gestos PAJ7620U2 — MATEIX bus que el color (GPIO2/3)
SDA_GEST = SDA_COL
SCL_GEST = SCL_COL
# INT del PAJ7620U2 NO connectat a la PCB -> el driver usa polling

# Bus I2C del sensor de distància VL53L0X (GPIO0/1, software i2c-gpio), sol al bus
SDA_DIST = 0
SCL_DIST = 1

# Finals de carrera (efecte Hall)
END_RA = 10   # braç dret     (pullup extern 10k a la PCB)
END_LA = 11   # braç esquerre (pullup extern 10k a la PCB)
END_SY = 12   # xeringa       (SENSE pullup extern -> cal PUD_UP per software)

# Sensors seguidors de línia (TCRT5000) -> canals de l'ADS1115, NO són GPIO
LINES_L = 0   # canal AIN0 de l'ADS1115
LINES_R = 1   # canal AIN1 de l'ADS1115

# ==================
# DISPLAY
# ==================

# Dades matriu LED 8x8 RGB WS2811 (2x ull)
# Originalment a GPIO9 (a la PCB), però GPIO9 NO té PWM/PCM/SPI per hardware i
# pigpio (1µs) no pot fer el timing WS2811 (el bit "0" surt com "1" -> tot blanc).
# Solucio: pont fisic GPIO13 -> GPIO9 (GPIO13 = PWM1, valid per a rpi_ws281x) i
# LED_DATA passa a 13. IMPORTANT: GPIO9 s'ha de deixar com a ENTRADA (alta
# impedancia) perque no lluiti amb GPIO13 (ho fa eyes_setup / LED_DATA_OLD).
LED_DATA     = 13   # nou pin de dades (PWM1), via pont a la PCB
LED_DATA_OLD = 9    # pin antic; cal deixar-lo en INPUT per no fer conflicte

# ==================
# ÀUDIO (afegit per company, no part d'aquest TFG)
# ==================

# I2S — amplificador + micròfon
I2C_BCLK  = 18
I2C_LRCLK = 19
MIC       = 20
AMP_DIN   = 21
