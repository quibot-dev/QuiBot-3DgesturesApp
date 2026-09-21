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
motion.py — Control de motors (rodes, braços, xeringa), sensor de distància
            VL53L0X i seguidor de línia TCRT5000 via ADS1115.
Equivalent a motion.cpp del codi Arduino/ESP32.
"""

import math
import time
import threading
import pigpio
import adafruit_extended_bus
import adafruit_vl53l0x
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

from pins import (
    STEP_R_W, DIR_R_W, STEP_L_W, DIR_L_W, EN_W,
    STEP_R_A, DIR_R_A, STEP_L_A, DIR_L_A, EN_A,
    STEP_SY,  DIR_SY,  EN_SERVO,
    END_SY, END_RA, END_LA,
)

# ==================
# Constants
# ==================

CW    = True
CCW   = False
ON    = 0       # A4988/TB6600: enable actiu en LOW
OFF   = 1
TAKE  = True
LEAVE = False

# Tipus de resposta del seguidor de línia
CLEAR    = 0
CROSSING = 1
OBJECT   = 2

# Noms de tasques (equivalent a les constants string del C++)
MOVE_TO_CROSSING  = "Move to crossing"
TURN_90_CW        = "Turn 90 CW"
TURN_90_CCW       = "Turn 90 CCW"
MOVE_TO_OBJECT    = "Move to object"
TAKE_SOMETHING    = "Take something"
LEAVE_SOMETHING   = "Leave something"
DO_NOTHING        = "Do nothing"

# Paràmetres de moviment
# 250 passos/s = 125 mm/s (1 mm = 2 passos). Acceleració 800: arriba a la
# velocitat en 0,31 s (uns 20 mm dels 300 del tram entre vèrtexs).
# El límit el posa el generador de polsos, no els motors: mesurat 2026-09-07
# amb la Raspberry nova, se satura a ~390 passos/s per roda (igual que
# l'anterior). Durant el seguiment de línia la roda de fora rep
# velocitat + correcció (màxim ~123 amb error 16000 i P_FACTOR=130), o sigui
# 250 + 123 = 373, que hi cap. A 350 en demanaria 473 i la correcció es
# retallaria en silenci: el robot giraria menys del que el control comanda.
WHEELS_MAX_SPEED  = 175.0   # steps/s
WHEELS_ACCEL      = 800.0   # steps/s²
ARMS_MAX_SPEED    = 250.0   # steps/s
ARMS_ACCEL        = 125.0   # steps/s²
# Velocitat de la xeringa. Baixada de 800 a 400 el 2026-09-10: movent aigua,
# el motor perdia passos. Un pas a pas perd parell a mesura que puja de
# revolucions, i alentir-lo és la primera palanca abans de tocar el Vref.
# Ajustada a 350 el 2026-09-10: amb aigua va bé i no cal tocar el Vref.
SYRINGE_MAX_SPEED = 350.0   # steps/s
SYRINGE_ACCEL     = 500.0   # steps/s²

WHEEL_MECH_REDUCTION       = 5
WHEEL_STEPS_PER_REVOLUTION = 200 * WHEEL_MECH_REDUCTION   # 1000 passos/volta de roda

# Distància que avança el robot des que detecta la cruïlla fins que l'eix de
# les rodes queda sobre el vèrtex, que és el punt sobre el qual ha de pivotar
# per encarar bé la línia perpendicular. Es compon de la separació física
# entre els sensors i l'eix (54 mm, mesurada) més el tram que queda des que
# el sensor entra a la cinta fins al centre d'aquesta (uns 6 mm).
# Ajustat de 62 a 60 el 2026-09-07 i de 60 a 57 el 2026-09-08: en el
# recorregut complet, el robot continuava avançant una mica de més abans de
# girar.
MM_TO_CROSSING_CENTER = 57
# Distància que avança el robot des del vèrtex fins a quedar davant del
# recipient, seguint la línia.
#
# El recipient sempre és al vèrtex següent i el robot sempre parteix d'un
# vèrtex, o sigui que aquesta distància és FIXA i es compta en passos. El
# sensor de distància NO s'usa per aturar-se: el seu paper és només dir si hi
# ha recipient o no (comprovació prèvia, amb el robot quiet), que és el que fa
# bé. Aturar-se pel sensor donava parades diferents cada vegada, perquè el
# punt on detectava variava amb la reflexió del tuper transparent.
# Calibrat al taulell el 2026-09-10: 140 mm.
MM_TO_OBJECT          = 140

# Distància a la qual es considera que hi ha un objecte al davant. Estava
# escrita dins de follow_line_loop() i no es podia ajustar sense editar el
# codi. ⚠️ El sensor de gestos treballa amb la mà a uns 15 cm: cal comprovar
# que una persona a prop no dispari aquesta detecció.
# Distància a la qual es considera que hi ha un objecte al davant.
#
# Pujada de 50 a 80 el 2026-09-10: la parada no era repetible. Com més a prop
# mesura el VL53L0X, més li afecten l'error relatiu i les variacions de
# reflexió del tuper (que a més és transparent). Detectant més lluny el senyal
# és més estable, i com que el tram que ve després és una distància fixa
# comptada en passos, el robot acaba sempre al mateix lloc.
OBJECT_DISTANCE_MM = 80

# Mínim físic del VL53L0X. Qualsevol mesura per sota d'aquest valor no és una
# distància, és una lectura invàlida: el sensor no pot mesurar tan a prop.
# Se n'han vist de 0 mm amb l'estat marcat com a bo.
DIST_MIN_VALID_MM = 30

# Una detecció d'objecte només es dona per bona si la mesura ANTERIOR ja el
# veia per sota d'aquest valor. Un objecte real no apareix de cop: acostant-s'hi
# a 75 mm/s amb una mesura cada ~195 ms, el robot només avança 15 mm entre
# lectures, de manera que abans d'estar a 80 mm ha d'haver passat per 150 i
# per 250. En canvi una lectura errònia salta de "fora de rang" a 150 mm d'una
# tirada. Aquesta condició les descarta sense endarrerir la parada, perquè amb
# un objecte real ja es compleix des de molt abans.
OBJECT_TRACK_MM = 250

# Comprovació prèvia: abans d'arrencar un tram, el robot mira si hi ha res al
# davant i, si no en veu, DESACTIVA la detecció durant tot aquell tram. Així
# cap lectura errònia el pot aturar enmig d'una casella buida.
#
# La gràcia és que aquesta comprovació es fa amb el robot ATURAT, o sigui que
# fer-hi diverses mesures no costa res: no avança mentre mesura. Es decideix
# per majoria, i això la fa fiable en els dos sentits —ni s'activa per una
# lectura falsa, ni es desactiva perquè justament aquella hagi fallat.
#
# Amb un tuper al vèrtex següent, el robot el veu a uns 150 mm des de la
# cruïlla; el llindar es posa molt més amunt per no dependre de la mida exacta
# del recipient, però per sota dels 300 mm de la casella següent.
# Finestra de validesa de la comprovació prèvia. Mesurat al taulell el
# 2026-09-08: amb el robot aturat a la cruïlla i un tuper al vèrtex següent,
# les lectures són molt estables entre 150 i 220 mm. Sense res al davant, el
# sensor dona 8090 (fora de rang) o valors erràtics de 0 a 100.
#
# ⚠️ La finestra és estreta a propòsit, i això la lliga a la MIDA i la
# COL·LOCACIÓ dels tupers actuals. Si es canvien per uns altres, o es
# col·loquen diferent, cal tornar a mesurar aquesta distància.
#
# Les dues bandes NO se solapen, i això és el que fa que aquest criteri sigui
# fiable: una lectura de 60 mm no és "un objecte molt a prop", és soroll,
# perquè amb el robot aturat a la cruïlla cap tuper hi pot ser tan a prop.
# Es descarta pel mateix motiu que es descarta un 8090.
#
# ⚠️ Aquesta finestra val NOMÉS per a la comprovació prèvia, amb el robot
# quiet. Durant la marxa el tuper s'acosta i les lectures baixen fins als
# 50 mm de la parada, de manera que allà no hi pot haver límit inferior.
OBJECT_AHEAD_MIN_MM = 140
OBJECT_AHEAD_MAX_MM = 220
OBJECT_AHEAD_SAMPLES = 3

# Es demana MAJORIA (2 de 3), no unanimitat. Els dos errors possibles no tenen
# la mateixa gravetat: amb la finestra, un fals positiu exigiria dues lectures
# errònies caient totes dues dins d'una banda on el soroll no hi va mai, cosa
# pràcticament impossible. En canvi, exigint-ne 3 n'hi hauria prou que UNA de
# les tres sortís errònia perquè el robot donés la casella per buida i es
# fiqués damunt del tuper.

# Es va provar un filtre de confirmació (declarar objecte només si 2 de les 3
# últimes mesures estaven per sota del llindar) i es va descartar: com que
# cada lectura costa ~195 ms i el robot avança mentre confirma, la parada
# passava a ser d'una a dues mesures més tard i la posició final deixava de
# ser predictible. Es treballa amb una sola lectura.

# Avanç per centrar l'eix sobre el vèrtex quan s'hi arriba DE RECULONS.
# És menor que MM_TO_CROSSING_CENTER perquè el robot troba la cinta per
# l'altra banda: anant endavant els sensors disparen entrant per la vora més
# llunyana del vèrtex, i anant enrere per la més propera. La diferència és
# aproximadament el gruix de la cinta menys el que els sensors s'hi endinsen
# abans de superar el llindar.
MM_BACKWARD_TO_CENTER = 54

# Velocitat màxima en marxa enrere. Calibrada al taulell el 2026-09-08:
# més ràpid va millor, i té sentit —com més estona dura el retrocés, més
# marge té la desviació per acumular-se.
WHEELS_BACKWARD_SPEED = 120.0

# Seguir la línia amb realimentació dels sensors quan es va enrere.
#
# Ha d'estar a False. El motiu és estructural: els sensors
# són DAVANT de l'eix de les rodes, o sigui que anant enrere queden a la cua.
# Amb l'estat (error del sensor, orientació) i un control proporcional, la
# matriu del sistema té traça -L·k i determinant v·k. Amb v negativa cap dels
# dos signes de k funciona: amb el d'endavant el determinant es fa negatiu
# (punt de sella) i amb l'invertit la traça es fa positiva (oscil·lació
# creixent). Comprovat al taulell el 2026-09-08: amb el signe invertit el
# robot es desvia molt de pressa quan és sobre la línia, i en canvi va recte
# quan l'ha perduda del tot, que és justament la firma d'aquest problema.
#
# Retrocedint sense realimentació el robot va recte. Com que arrenca ben
# alineat —acaba de centrar-se al vèrtex— i només ha de fer 300 mm, la
# desviació és de pocs mil·límetres i els sensors detecten igualment la cinta
# de la cruïlla en passar-hi. La compensació de LINE_ERROR_TRIM sí que es
# manté: no és realimentació, és equilibrat mecànic de les rodes.
BACKWARD_LINE_FOLLOW = False

# Guany de la correcció en marxa enrere. És un divisor, o sigui que un valor
# MÉS GRAN vol dir correcció més suau. Ha de ser força més gran que el
# d'endavant: la divergència del llaç amb els sensors a la cua creix com
# l'arrel del guany, de manera que suavitzar-lo la fa prou lenta perquè no
# arribi a molestar en els 300 mm d'una casella.
P_FACTOR_BACKWARD = 400

# Compensació de la deriva mecànica en marxa enrere, independent de la
# d'endavant. No n'hi ha prou de canviar el signe: la bola de suport es
# comporta diferent quan arrossega que quan empeny, i això no ho compensa un
# equilibrat de rodes. Es calibra a part, amb l'opció 4 del test.
LINE_ERROR_TRIM_BACKWARD = 750

# Inverteix el signe de la correcció en marxa enrere.
#
# Segons la geometria, el signe correcte és el mateix que endavant: si el
# sensor dret toca la cinta, la roda dreta ha d'accelerar (en marxa enrere)
# perquè l'extrem dels sensors es desplaci cap a la cinta. Però el llaç és
# inestable en aquest sentit, i en un llaç inestable la correcció instantània
# pot ser correcta mentre el resultat acumulat va en contra: vist de fora és
# indistingible de tenir el signe girat. Es deixa com a interruptor per
# decidir-ho provant, no discutint.
# Comprovat al taulell el 2026-09-08: si es volgués realimentar, el signe bo
# seria l'INVERTIT. Els dos són inestables, però de manera diferent —amb el
# normal la desviació fuig de forma monòtona i amb l'invertit el que creix és
# una oscil·lació—, i sobre els 300 mm d'una casella l'oscil·lació encara no
# ha tingut temps de fer-se gran. Tot i així es va decidir NO realimentar: amb
# aquestes distàncies no hi ha marge per corregir de veritat, i el que compta
# és arrencar recte i mantenir-se recte.
BACKWARD_INVERT_CORRECTION = True

# Llindar de negre per ADS1115 (GAIN_ONE ±4.096V, single-ended 0–26400 per a 3.3V).
# Calibrat 2026-08-28 sobre el taulell real (lona + cinta): blanc 1000–1200,
# cobertura parcial de la cinta 5000–12000, cruïlla ~18000 als dos sensors.
# Només s'usa per detectar cruïlles (els dos sensors negres alhora); la correcció
# de direcció treballa amb la diferència en cru. Per això el llindar va alt: ha de
# quedar per damunt de la cobertura parcial per no cantar cruïlles falses.
# Llindar únic, comú a la detecció de cruïlles, al gir i a la marxa enrere.
#
# Baixat de 12000 a 10000 el 2026-09-08 a partir dels registres de les proves.
# El que decideix és el MENOR dels dos sensors (cal que tots dos el passin).
# Sobre ~4000 lectures fora de cruïlla dels 26 registres de logs-linia/:
#   - mediana 1.186 i p99 1.313  -> la lona blanca
#   - només 16 mostres passen de 5.000, totes d'un mateix episodi de ~0,6 s
#     amb el robot molt desviat; el màxim d'aquell episodi és 8.281
#   - mínim observat en una cruïlla real: 12.269
# 10000 queda molt per damunt del blanc i del pitjor episodi, i ~2.300 per
# sota de la cruïlla més fluixa.
# (Revisat 2026-09-17. El 14.350 anterior sortia només de la sessió del 28-08.)
BLACK_THRESHOLD = 10000

# Divisor de la correcció proporcional del seguidor de línia. L'error (lf_r - lf_l)
# arriba a ~17000 amb un sensor sobre la cinta; amb WHEELS_MAX_SPEED = 130, un
# divisor de 130 deixa la correcció màxima a ≈130 (una roda s'atura, l'altra dobla).
# El valor heretat de l'ESP32 era 5, pensat per a un ADC de 12 bits (0–4095).
P_FACTOR = 130

# Cadència màxima que el generador de polsos lliura DE VERITAT durant el
# seguiment de línia. No s'ha de confondre amb els ~390 passos/s mesurats amb
# el robot aixecat i el fil principal aturat: mentre se segueix la línia, el
# fil principal llegeix l'ADS1115 unes 43 vegades per segon i li pren CPU al
# generador, que baixa a 170-300 passos/s (mesurat 2026-09-07 comparant els
# passos comandats amb els realment executats).
#
# Comandar per sobre d'aquest valor no fa anar la roda més ràpid, però sí que
# destrueix el gir: totes dues rodes queden limitades per la cadència del
# bucle i el diferencial entre elles desapareix justament quan la correcció
# és més gran. És la causa que el robot es desviés sense recuperar-se.
MAX_STEP_RATE = 200.0

# Cadència mínima de la roda interior mentre el robot corregeix. Si es deixa
# arribar a zero, el robot pivota sense avançar i es queda vibrant dins de la
# zona negra sense estabilitzar-se: és el moviment cap endavant el que
# esmorteeix el llaç. Limitant la roda interior es limita el diferencial
# màxim a MAX_STEP_RATE - MIN_WHEEL_RATE, que és el preu a pagar.
MIN_WHEEL_RATE = 50.0

# Biaix constant que se suma a l'error dels sensors de línia. Serveix per
# compensar una deriva mecànica sistemàtica (rodes de diàmetre lleugerament
# diferent, motors que no responen igual, fregament de la bola de suport).
# Valor negatiu = el robot tendeix a girar més a l'esquerra.
# Es calibra amb l'opció 4 del test: si el robot se'n va sempre cap a la
# dreta, cal fer-lo més negatiu. Calibrat al taulell el 2026-09-07: amb -1000
# (uns 8 passos/s de diferencial constant) el robot deixa de derivar.
LINE_ERROR_TRIM = -1000

# Llindar d'error de rotació (equivalent a 20/4095 de l'ESP32 → ~130 en ADS1115).
ROTATION_ERROR_THRESHOLD = 130

# Fracció de volta de roda que fa un gir de 90°, en tant per cent.
# El valor heretat era 42, calculat suposant una via entre rodes de 250 mm.
# La via real és de 267 mm, cosa que ja dona 44; a més, en un gir sobre si
# mateix les rodes freguen de costat i se'n perd una part. Mesurat al taulell
# el 2026-09-08: amb 42 el gir es quedava curt fins i tot arribant al 110%.
# És el punt de partida de la fase a cegues; l'ajust fi amb sensors acaba de
# tancar el gir sobre la línia.
ROTATION_PERCENT = 46

# Finestra de l'ajust fi, en tant per cent dels passos de rotació.
#
# L'inici ha de ser prou d'hora perquè el PRIMER sensor que creua la cinta
# perpendicular ho faci ja dins de la finestra. Si comença massa tard, en un
# dels dos sentits de gir aquella primera passada queda fora, l'algorisme
# espera la del segon sensor i el gir se'n va uns 30° de més. Amb l'inici al
# 85% això passava girant a l'esquerra: 534-547 passos contra els 438-474 de
# la dreta (mesurat al taulell 2026-09-08).
ROTATION_FINE_START = 60
ROTATION_FINE_LIMIT = 130

# Passos addicionals després de detectar el final del gir. Normalment ha de
# ser 0: el criteri de parada ja acaba al punt centrat. Es deixa com a ajust
# per si calgués corregir un biaix residual.
ROTATION_FINISH_STEPS = 0

# Posicions dels braços en passos des del home
# Posicions dels braços, en passos des del topall mecànic de baix, que és la
# referència que el homing pot retrobar sol. Mesurades al robot el 2026-09-09:
#
#     0    topall de baix
#    40    posició de xuclar (calibrada amb els tupers reals)
#   170    posició de repòs amb els motors apagats (la donen les molles)
#   335    posició alta: els braços deixen lliure el sensor de gestos
#   345    topall de dalt
#
# ⚠️ Les molles NO aguanten la posició alta: els motors dels braços han de
# quedar energitzats (EN_A actiu) mentre el robot espera gestos, o cauran i es
# perdrà la posició. Per això arms_home() no desactiva els braços en acabar.
ARM_LOWER_POSITION   = 40
ARM_L_UPPER_POSITION = 335
ARM_R_UPPER_POSITION = 335

# Recorregut a cegues del homing dels braços.
#
# Ha de cobrir el recorregut complet (345) amb marge, perquè el homing no sap
# on són els braços quan comença: si s'atura i es torna a engegar el programa
# sense tallar el corrent, es queden amunt (els GPIO mantenen l'estat) mentre
# que el comptador del programa nou arrenca a zero. 370 hi arriba des de
# qualsevol posició amb 25 passos de marge.
#
# ⚠️ Es va provar de desactivar els braços mig segon abans de començar, perquè
# caiguessin a una posició de repòs coneguda i així escurçar el recorregut a
# 250. DESCARTAT el 2026-09-09: en treure les molles, aquella caiguda passa a
# ser lliure contra el topall. El valor heretat era 1250.
ARMS_HOMING_STEPS = 370

# Xeringa: 10 rev * 200 passos/rev * microstepping x4 = 8000 passos estesa del tot
# Recorregut útil de la xeringa, en passos. MESURAT al robot el 2026-09-09:
# el topall és a 2800, però se'n fan servir 2500: és més volum del que cal
# per als tupers i deixa marge de sobres abans del final.
#
# El valor anterior era 8000, d'un càlcul teòric heretat de l'ESP32
# (10 voltes × 200 passos × 4 de microstepping) que ningú havia comprovat.
# Amb aquell número, cada xuclada enviava 5200 passos contra el final.
_SY_FULL_EXTENDED_STEPS = 2500

# Recorregut a cegues del homing de la xeringa.
#
# ⚠️ La xeringa NO fa homing a l'arrencada: acaba sempre a zero després de
# cada cicle i, comprovat el 2026-09-09, el pistó no rellisca amb els motors
# apagats. La referència es manté sola. La funció es conserva per poder
# recalibrar sota demanda (des de la web) si algun cop s'apaga el robot amb
# la xeringa a mig recorregut, cas en què el comptador diria zero sense ser-hi.
#
# El valor heretat era 11000 sobre un recorregut real de 2800.
SYRINGE_HOMING_STEPS = 2900

LINE_FOLLOWER_FREQ   = 100              # Hz (objectiu; mesura la real amb el test)
LINE_FOLLOWER_PERIOD = 1.0 / LINE_FOLLOWER_FREQ  # s

# Velocitat de mostreig de l'ADS1115. El valor per defecte de la llibreria és
# 128 SPS, que costa ~8 ms de conversió per canal i és el que ofegava el bucle
# del seguidor. A 860 SPS baixa a ~1,2 ms; el soroll addicional és irrellevant
# perquè blanc (~1100) i negre (~18000) estan separats un factor 10.
ADS_DATA_RATE = 860

# El VL53L0X penja d'un bus I²C per software (bit-bang, GPIO0/1), que és lent:
# una lectura costa ~100 ms i, feta a cada iteració, limitava el seguidor a
# 8 Hz. Es llegeix una vegada de cada N iteracions i entremig es reutilitza
# l'última mesura. Amb N=5 el cost es reparteix i la detecció de vasos manté
# resolució de sobres (a 150 mm/s són ~20 mm entre lectures).
DIST_CHECK_EVERY = 5

# ⚠️ TEMPORAL (2026-08-28): detecció d'objectes DESACTIVADA al seguidor de línia.
# A la primera prova al taulell el robot es va aturar per un fals positiu del
# VL53L0X (OBJECT amb res al davant) abans d'arribar a cap cruïlla. Mentre no
# s'esbrini, follow_line_loop() no retorna mai OBJECT i el seguidor només mira
# cruïlles. La distància se segueix mesurant i registrant (lf_last_dist), així
# que els CSV de les proves permetran veure quan i amb quin valor passa.
# Tornar a posar True quan estigui resolt: el mode de vasos ho necessita.
DETECT_OBJECT = True


# ==================
# Classe Stepper
# ==================

class Stepper:
    """
    Motor pas a pas en mode DRIVER (STEP/DIR).
    Equivalent a AccelStepper(DRIVER, step_pin, dir_pin).
    Genera polsos STEP via pigpio.gpio_trigger().
    """

    PULSE_US = 10   # Amplada del pols STEP en µs (A4988 requereix ≥1µs)

    def __init__(self, pi: pigpio.pi, step_pin: int, dir_pin: int,
                 invert: bool = False):
        self._pi        = pi
        self._step_pin  = step_pin
        self._dir_pin   = dir_pin
        self._invert    = bool(invert)   # inverteix el sentit de gir (motor muntat mirall)
        self._dir_level = 0       # últim nivell escrit al pin DIR (evita escriure'l cada pas)
        self._pos       = 0       # posició actual (passos)
        self._target    = 0       # posició objectiu
        self._speed     = 0.0     # velocitat actual (passos/s, signada)
        self._max_speed = 1.0
        self._accel     = 1.0
        self._last_step_us     = self._now_us()
        self._step_interval_us = 0   # 0 = aturat

        pi.set_mode(step_pin, pigpio.OUTPUT)
        pi.set_mode(dir_pin,  pigpio.OUTPUT)
        pi.write(step_pin, 0)
        pi.write(dir_pin,  0)   # coherent amb self._dir_level = 0

    @staticmethod
    def _now_us() -> int:
        return time.monotonic_ns() // 1000

    def set_max_speed(self, speed: float):
        self._max_speed = abs(speed)

    def set_acceleration(self, accel: float):
        self._accel = abs(accel)

    def move_to(self, position: int):
        self._target = int(position)

    def move(self, relative: int):
        self._target = self._pos + int(relative)

    def set_current_position(self, pos: int):
        self._pos    = int(pos)
        self._target = int(pos)
        self._speed  = 0.0
        self._step_interval_us = 0

    def current_position(self) -> int:
        return self._pos

    def distance_to_go(self) -> int:
        return self._target - self._pos

    def is_running(self) -> bool:
        return self._target != self._pos

    def stop(self):
        self._target = self._pos
        self._speed  = 0.0
        self._step_interval_us = 0

    def set_speed(self, speed: float):
        """Estableix velocitat constant per a run_speed()."""
        self._speed = float(speed)
        self._step_interval_us = int(1_000_000 / abs(speed)) if speed != 0.0 else 0

    def _do_step(self, direction: int):
        # Cada crida a pigpio és un viatge d'anada i tornada per socket fins al
        # dimoni (~1,2 ms), i és el que limita la cadència de passos. El pin DIR
        # només canvia quan el motor inverteix el sentit, així que només s'escriu
        # quan cal: estalvia la meitat de les crides. Mesurat 2026-08-28: amb
        # dues crides per pas el sostre eren 205 passos/s per roda.
        level = 1 if direction > 0 else 0
        if self._invert:
            level = 1 - level
        if level != self._dir_level:
            self._pi.write(self._dir_pin, level)
            self._dir_level = level
        self._pi.gpio_trigger(self._step_pin, self.PULSE_US, 1)
        self._pos += direction

    def run_speed(self, now: int = None) -> bool:
        """
        Fa un pas a velocitat constant. No bloquejant — cridar des del bucle de
        steppers.

        `now` permet passar el temps ja llegit: el bucle el llegeix UNA vegada
        per volta en lloc d'una per motor. La lectura del rellotge és barata,
        però a desenes de milers de voltes per segon i cinc motors, deixa de
        ser-ho.
        """
        if self._step_interval_us == 0:
            return False
        if now is None:
            now = self._now_us()
        if now - self._last_step_us >= self._step_interval_us:
            self._do_step(1 if self._speed > 0 else -1)
            self._last_step_us = now
            return True
        return False

    def run(self, now: int = None) -> bool:
        """
        Fa un pas cap a _target amb acceleració/desacceleració.
        No bloquejant — cridar des del bucle de steppers.
        Implementa l'algorisme de AccelStepper: Δv = accel / v per pas.
        """
        dtg = self.distance_to_go()
        if dtg == 0:
            self._speed = 0.0
            self._step_interval_us = 0
            return False

        abs_speed = abs(self._speed)
        if abs_speed < 1.0:
            abs_speed = math.sqrt(self._accel / 2.0)   # velocitat inicial AccelStepper

        if now is None:
            now = self._now_us()
        if now - self._last_step_us < int(1_000_000 / abs_speed):
            return False   # no és hora del proper pas

        # Actualitza la velocitat per al proper pas
        direction  = 1 if dtg > 0 else -1
        stop_dist  = (abs_speed ** 2) / (2.0 * self._accel) if self._accel > 0 else 0
        if abs(dtg) <= max(stop_dist, 1):
            new_speed = abs_speed - (self._accel / abs_speed)
            new_speed = max(new_speed, 1.0)
        else:
            new_speed = abs_speed + (self._accel / abs_speed)
            new_speed = min(new_speed, self._max_speed)

        self._speed = new_speed * direction
        self._do_step(direction)
        self._last_step_us = now
        return True


# ==================
# Instàncies globals
# ==================

_pi:          pigpio.pi = None
_i2c_dist               = None
_i2c_ads                = None
_dist_sensor            = None
_ads                    = None
_chan_r:      AnalogIn  = None
_chan_l:      AnalogIn  = None

wheel_R: Stepper = None
wheel_L: Stepper = None
arm_R:   Stepper = None
arm_L:   Stepper = None
syringe: Stepper = None

wheels_speed_mode: bool = False   # True → run_speed(), False → run() (posició)

_stepper_stop   = threading.Event()
_stepper_thread: threading.Thread = None


# ==================
# Setup i cleanup
# ==================

def motion_setup_steppers(pi: pigpio.pi):
    """
    Inicialitza GPIOs i steppers sense els sensors I2C.
    Útil per a tests de motors quan VL53L0X/ADS1115 no estan connectats.
    """
    global _pi, wheel_R, wheel_L, arm_R, arm_L, syringe, _stepper_thread

    _pi = pi

    for pin in (EN_W, EN_A, EN_SERVO):
        pi.set_mode(pin, pigpio.OUTPUT)
    for pin in (END_LA, END_RA, END_SY):
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)   # Hall actiu-baix, pull-up intern

    enable_arms(OFF)
    enable_wheels(OFF)

    # Convenció: positiu = ENDAVANT (rodes) i positiu = AMUNT (braços).
    # Els motors muntats en mirall respecte el seu parell porten invert=True.
    # Verificat al robot 2026-08-28: la roda esquerra girava al revés de la
    # convenció. Als braços, invertir el dret els va sincronitzar però tots dos
    # anaven avall amb ordre positiva, així que la inversió va a l'esquerre.
    wheel_R = Stepper(pi, STEP_R_W, DIR_R_W)
    wheel_L = Stepper(pi, STEP_L_W, DIR_L_W, invert=True)
    arm_R   = Stepper(pi, STEP_R_A, DIR_R_A)
    arm_L   = Stepper(pi, STEP_L_A, DIR_L_A, invert=True)
    syringe = Stepper(pi, STEP_SY,  DIR_SY)

    wheel_R.set_max_speed(WHEELS_MAX_SPEED);  wheel_R.set_acceleration(WHEELS_ACCEL)
    wheel_L.set_max_speed(WHEELS_MAX_SPEED);  wheel_L.set_acceleration(WHEELS_ACCEL)
    arm_R.set_max_speed(ARMS_MAX_SPEED);      arm_R.set_acceleration(ARMS_ACCEL)
    arm_L.set_max_speed(ARMS_MAX_SPEED);      arm_L.set_acceleration(ARMS_ACCEL)
    syringe.set_max_speed(SYRINGE_MAX_SPEED); syringe.set_acceleration(SYRINGE_ACCEL)

    _stepper_stop.clear()
    _stepper_thread = threading.Thread(target=_stepper_loop, daemon=True, name="steppers")
    _stepper_thread.start()


# Busos I2C (segons /boot/firmware/config.txt):
#   /dev/i2c-1 (hardware, GPIO2/3): TCS34725, PAJ7620U2, ADS1115
#   /dev/i2c-3 (software i2c-gpio, GPIO0/1): VL53L0X
I2C_BUS_ADS  = 1   # ADS1115 (sensors de línia) — bus hardware GPIO2/3
I2C_BUS_DIST = 3   # VL53L0X (distància) — bus software GPIO0/1


def motion_setup_sensors(pi: pigpio.pi):
    """
    Inicialitza únicament els sensors I2C. Sense steppers.
    Útil per a tests de sensors quan els motors no estan connectats.

    Amb la PCB definitiva, el VL53L0X i l'ADS1115 estan en busos DIFERENTS:
    el VL53L0X sol al bus software GPIO0/1, i l'ADS1115 al bus hardware GPIO2/3.
    """
    global _pi, _i2c_dist, _i2c_ads, _dist_sensor, _ads, _chan_r, _chan_l

    _pi = pi

    _i2c_dist    = adafruit_extended_bus.ExtendedI2C(I2C_BUS_DIST)
    _dist_sensor = adafruit_vl53l0x.VL53L0X(_i2c_dist)

    _i2c_ads     = adafruit_extended_bus.ExtendedI2C(I2C_BUS_ADS)
    _ads         = ADS.ADS1115(_i2c_ads)
    _ads.data_rate = ADS_DATA_RATE     # per defecte 128 SPS -> ~8 ms per lectura
    _chan_r      = AnalogIn(_ads, 0)   # LINES_R = canal 0 (AIN0)
    _chan_l      = AnalogIn(_ads, 1)   # LINES_L = canal 1 (AIN1)


def motion_setup(pi: pigpio.pi):
    """
    Inicialitza GPIOs, steppers, sensor de distància VL53L0X,
    ADC ADS1115 per als sensors de línia, i arrenca el bucle de steppers.
    Requereix a /boot/firmware/config.txt:
        dtparam=i2c_arm=on
        dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=0,i2c_gpio_scl=1
    """
    motion_setup_steppers(pi)
    motion_setup_sensors(pi)


def motion_cleanup(lower_arms: bool = True):
    """
    Baixa els braços, atura el bucle de steppers i desactiva tots els motors.

    Els braços es baixen abans d'apagar perquè, sense molles, en tallar el
    corrent cauen de cop contra el topall. Baixar-los controladament els deixa
    al zero, que a més és la posició des d'on el programa següent espera
    trobar-los.
    """
    if lower_arms and arm_L is not None:
        try:
            enable_arms(ON)
            move_arms_to(ARM_LOWER_POSITION)
        except Exception:
            pass          # mai bloquejar l'apagada per un error aquí
    _stepper_stop.set()
    if _stepper_thread:
        _stepper_thread.join(timeout=1.0)
    enable_wheels(OFF)
    enable_arms(OFF)
    enable_syringe(OFF)


# ==================
# Enable / disable
# ==================

def enable_wheels(state: bool):
    _pi.write(EN_W, state)

def enable_arms(state: bool):
    _pi.write(EN_A, state)

def enable_syringe(state: bool):
    _pi.write(EN_SERVO, state)

def is_endstop_detecting(pin: int) -> bool:
    return not _pi.read(pin)   # efecte Hall actiu en baix


# ==================
# Helpers de moviment
# ==================

def mm_to_steps(mm: int) -> int:
    # Perímetre roda Ø152mm = 2·π·76 ≈ 477mm
    return (mm * WHEEL_STEPS_PER_REVOLUTION * 2) // 1000

def wheels_set_position(position: int = 0):
    wheel_L.set_current_position(position)
    wheel_R.set_current_position(position)

def wheels_set_speed(speed: float, rotate: bool = False, direction: bool = CW):
    if rotate:
        wheel_L.set_speed( speed if direction == CW else -speed)
        wheel_R.set_speed(-speed if direction == CW else  speed)
    else:
        wheel_L.set_speed(speed)
        wheel_R.set_speed(speed)

def move_arms_to(position: int):
    arm_L.move_to(position)
    arm_R.move_to(position)
    while (arm_L.distance_to_go() or arm_R.distance_to_go()) and not cancelled():
        time.sleep(0.1)

def move_arms_up():
    arm_L.move_to(ARM_L_UPPER_POSITION)
    arm_R.move_to(ARM_R_UPPER_POSITION)
    while (arm_L.distance_to_go() or arm_R.distance_to_go()) and not cancelled():
        time.sleep(0.1)

def move_wheels_to(position: int, invert: bool = False):
    wheel_L.move_to(position)
    wheel_R.move_to(-position if invert else position)
    while wheel_L.is_running() and wheel_R.is_running() and not cancelled():
        time.sleep(0.1)


# ==================
# Homing
# ==================

def arms_home():
    """
    Cicle de homing dels braços (bloquejant). El bucle de steppers fa el
    moviment.

Sense finals de carrera, la referència és el topall mecànic de baix i
    el recorregut a cegues ha de ser prou llarg per arribar-hi vinguin d'on
    vinguin els braços.
    """
    enable_arms(ON)
    arm_L.move(-ARMS_HOMING_STEPS)
    arm_R.move(-ARMS_HOMING_STEPS)

    while not cancelled():
        if is_endstop_detecting(END_LA):
            arm_L.stop()
        if is_endstop_detecting(END_RA):
            arm_R.stop()
        l_done = is_endstop_detecting(END_LA) or arm_L.distance_to_go() == 0
        r_done = is_endstop_detecting(END_RA) or arm_R.distance_to_go() == 0
        if l_done and r_done:
            break
        time.sleep(0.005)

    arm_L.set_current_position(0)
    arm_R.set_current_position(0)
    arm_L.move(ARM_L_UPPER_POSITION)
    arm_R.move(ARM_R_UPPER_POSITION)

    while (arm_L.distance_to_go() or arm_R.distance_to_go()) and not cancelled():
        time.sleep(0.01)


def syringe_home():
    """Cicle de homing de la xeringa (bloquejant). El bucle de steppers fa el moviment."""
    enable_syringe(ON)
    syringe.move(-SYRINGE_HOMING_STEPS)

    while not is_endstop_detecting(END_SY):
        if syringe.distance_to_go() == 0:
            break
        time.sleep(0.005)

    syringe.stop()
    syringe.set_current_position(0)
    enable_syringe(OFF)


# ==================
# Sensor de distància
# ==================

def distance_to_object() -> int:
    """
    Retorna la distància en mm a l'objecte més proper, o 65535 si fora de rang.

    Replica el codi original (ESP32): si la mesura no és vàlida
    (range_status != 0) es considera "fora de rang" i es retorna 65535,
    per evitar lectures de soroll quan no hi ha cap objectiu clar al davant.
    """
    try:
        d = _dist_sensor.range
        status = getattr(_dist_sensor, "range_status", 0)
        if status != 0:        # mesura no vàlida -> fora de rang
            return 65535
        if d < DIST_MIN_VALID_MM:
            return 65535       # per sota del mínim físic del sensor
        return d
    except Exception:
        return 65535


# ==================
# Seguidor de línia
# ==================

# Última lectura dels sensors de línia, desada per follow_line_loop().
# Permet als tests registrar els valors reals sense fer una segona lectura I2C.
lf_last_r: int = 0
lf_last_l: int = 0
rotation_ended_on_line: bool = False
lf_last_dist: int = 65535   # última distància mesurada (mm), 65535 = fora de rang
_dist_prev:   int = 65535   # la immediatament anterior, per validar la detecció
_object_expected: bool = False   # hi ha objecte al davant en aquest tram?
at_object: bool = False          # el robot està aturat davant d'un recipient

# True mentre s'executa una acció de moviment. Els bucles de sondeig (gestos,
# blocs, render dels ulls) el consulten per cedir CPU: en una Raspberry d'un
# sol nucli, el generador de polsos competeix amb tot el que faci el programa,
# i cada lectura I²C o cada render li roba temps. Amb el robot aturat no
# importa; movent-se, sí.
moving: bool = False


# Senyal de cancel·lació. Les accions de moviment el consulten als seus bucles
# i en surten immediatament quan s'activa.
#
# Sense això, aturar el robot deixava les rodes parades però el bucle de
# l'acció seguia esperant una cruïlla que no arribaria mai: mantenia el lock
# agafat i el robot quedava bloquejat per sempre.
_cancel = threading.Event()


def cancel_action():
    """Demana que l'acció en curs s'aturi. La crida el botó d'aturada."""
    _cancel.set()


def cancelled() -> bool:
    return _cancel.is_set()
_dist_counter: int = 0


def _read_line_sensors() -> tuple:
    """Llegeix sensors de línia via ADS1115. Retorna (dreta, esquerra), 0–32767."""
    return _chan_r.value, _chan_l.value

def pace_line_follower(t_inici: float):
    """
    Completa el període del seguidor de línia només si sobra temps.

    Abans es feia un `sleep(LINE_FOLLOWER_PERIOD)` incondicional a cada volta.
    Com que una iteració ja costa força més que el període objectiu (mesurat
    2026-09-07: 46 ms contra els 10 ms de disseny), aquella espera era temps
    regalat que reduïa encara més la freqüència de mostreig.
    """
    resta = LINE_FOLLOWER_PERIOD - (time.monotonic() - t_inici)
    # Límit de seguretat: t_inici ha de venir de time.monotonic(). Si algú hi
    # passa un temps d'un altre rellotge (time.time(), per exemple), la resta
    # dona un valor absurd i el bucle es quedaria aturat indefinidament.
    if resta > 0:
        time.sleep(min(resta, LINE_FOLLOWER_PERIOD))


_last_accel_t = None

def reset_acceleration():
    """
    Reinicia el rellotge d'acceleració. Cridar just abans d'arrencar un tram.
    """
    global _last_accel_t, lf_last_dist, _dist_prev
    _last_accel_t = None
    lf_last_dist = _dist_prev = 65535   # cap objecte arrossegat del tram anterior

def compute_new_speed(speed: float) -> float:
    """
    Aplica WHEELS_ACCEL durant el temps REALMENT transcorregut des de l'última
    crida. Abans es dividia l'acceleració per LINE_FOLLOWER_FREQ (100 Hz), però
    el bucle no hi arriba: mesurat 8 Hz amb la lectura de distància a cada volta.
    Amb la divisió fixa, l'acceleració efectiva quedava 12 vegades més petita i
    el robot no arribava mai a WHEELS_MAX_SPEED en un tram curt.
    """
    global _last_accel_t
    now = time.monotonic()
    if _last_accel_t is None:
        dt = LINE_FOLLOWER_PERIOD
    else:
        dt = min(now - _last_accel_t, 0.2)   # límit per si el bucle s'ha aturat
    _last_accel_t = now
    return min(speed + WHEELS_ACCEL * dt, WHEELS_MAX_SPEED)

class _Moving:
    """
    Marca `motion.moving` mentre dura una acció de moviment.

    Els bucles de sondeig del programa (gestos, blocs, render dels ulls) el
    consulten per cedir CPU: en una Raspberry d'un sol nucli el generador de
    polsos competeix amb tot el que faci el procés, i cada lectura I²C li roba
    temps. Amb el robot aturat no importa; movent-se, sí — mesurat: el sostre
    de cadència baixa notablement amb quibot.py corrent.
    """

    def __enter__(self):
        global moving
        _cancel.clear()      # cada acció comença sense cancel·lació pendent
        moving = True
        return self

    def __exit__(self, *exc):
        global moving
        moving = False
        return False


def check_object_ahead() -> bool:
    """
    Mira, amb el robot ATURAT, si hi ha alguna cosa al davant, i habilita o
    desactiva la detecció d'objectes per al tram que ve. Cridar just abans
    d'arrencar, mai en moviment.
    """
    global _object_expected
    if not DETECT_OBJECT:
        _object_expected = False
        return False
    lectures = [distance_to_object() for _ in range(OBJECT_AHEAD_SAMPLES)]
    valides = sum(1 for d in lectures
                  if OBJECT_AHEAD_MIN_MM <= d <= OBJECT_AHEAD_MAX_MM)
    _object_expected = valides > OBJECT_AHEAD_SAMPLES // 2
    return _object_expected


def follow_line_loop(speed: float, forward: bool = True,
                     detect_crossing: bool = True,
                     detect_object: bool = True) -> int:
    """
    Seguidor de línia no bloquejant. Retorna CLEAR, CROSSING o OBJECT.
    Si CLEAR, aplica correcció proporcional a les velocitats de les rodes.
    """

    lf_r, lf_l = _read_line_sensors()
    global lf_last_r, lf_last_l, lf_last_dist, _dist_counter, _dist_prev
    lf_last_r, lf_last_l = lf_r, lf_l

    # El bus del VL53L0X és bit-bang i cada lectura costa ~80 ms: només se'n fa
    # una de cada DIST_CHECK_EVERY i entremig es reutilitza l'última. I només
    # es llegeix si la comprovació prèvia ha vist un objecte en aquest tram:
    # si la casella és buida, la lectura frenaria el seguidor per a res.
    if DETECT_OBJECT and _object_expected:
        _dist_counter += 1
        if _dist_counter >= DIST_CHECK_EVERY:
            _dist_counter = 0
            _dist_prev, lf_last_dist = lf_last_dist, distance_to_object()

    # Amb detect_crossing=False la funció només corregeix la trajectòria i mai
    # retorna CROSSING. Cal quan el robot ARRENCA damunt d'una cruïlla: si no,
    # la detecció salta a la primera iteració, la funció retorna abans de posar
    # cap velocitat a les rodes i el robot no es mou mai.
    if detect_crossing and forward and \
            lf_l > BLACK_THRESHOLD and lf_r > BLACK_THRESHOLD:
        return CROSSING
    elif detect_object and DETECT_OBJECT and _object_expected and forward and \
            lf_last_dist < OBJECT_DISTANCE_MM and _dist_prev < OBJECT_TRACK_MM:
        return OBJECT
    else:
        # La compensació mecànica de la deriva i la realimentació dels sensors
        # es tracten per separat: la primera és un equilibrat de rodes i la
        # segona és el control. Cadascuna té els seus paràmetres per a la
        # marxa enrere, perquè cap de les dues s'hi comporta com endavant.
        err  = lf_r - lf_l
        trim = LINE_ERROR_TRIM if forward else LINE_ERROR_TRIM_BACKWARD
        pfac = P_FACTOR if forward else P_FACTOR_BACKWARD

        d = 2 * (trim // P_FACTOR)
        if forward:
            d += 2 * (err // pfac)
        elif BACKWARD_LINE_FOLLOW:
            corr = 2 * (err // pfac)
            d += -corr if BACKWARD_INVERT_CORRECTION else corr

        # El diferencial es limita perquè la roda interior no baixi de
        # MIN_WHEEL_RATE: el robot ha de continuar avançant mentre corregeix.
        d_max = MAX_STEP_RATE - MIN_WHEEL_RATE
        d = max(-d_max, min(d_max, d))

        base = speed if forward else -speed
        v_l  = base + d / 2
        v_r  = base - d / 2

        # La correcció es reparteix simètricament entre les dues rodes: la
        # velocitat mitjana no canvia i el robot continua avançant mentre
        # corregeix, que és el que esmorteeix el llaç. Frenar només la roda
        # interior es va provar el 2026-09-07 i va sortir pitjor: en perdre
        # avanç, el robot es queda dins de la pertorbació i oscil·la.
        #
        # Si tot i així la roda de fora demanés més del que el generador de
        # polsos pot lliurar, es desplacen TOTES DUES velocitats en lloc de
        # retallar-ne una de sola. Retallar-ne una destrueix el diferencial
        # —i per tant el gir— justament quan la correcció és més gran; això
        # era la causa que el robot es desviés sense recuperar-se.
        if forward:
            exces = max(v_l, v_r) - MAX_STEP_RATE
            if exces > 0:
                v_l -= exces
                v_r -= exces
        else:
            exces = -MAX_STEP_RATE - min(v_l, v_r)
            if exces > 0:
                v_l += exces
                v_r += exces

        wheel_L.set_speed(v_l)
        wheel_R.set_speed(v_r)
        return CLEAR

def run_to_crossing_center(speed: float) -> float:
    """
    Avança per centrar el robot sobre el creuament detectat.
    Retorna la nova velocitat (reduïda a la meitat).
    """
    wheels_set_position(0 - mm_to_steps(MM_TO_CROSSING_CENTER))

    while follow_line_loop(speed) == CROSSING:
        wheels_set_speed(speed)
        time.sleep(0.1)

    speed /= 2
    while wheel_L.current_position() < 0 and wheel_R.current_position() < 0 \
            and not cancelled():
        wheels_set_speed(speed)
        time.sleep(0.1)
    wheels_set_speed(0)
    return speed


# ==================
# Bucle de steppers (thread permanent)
# ==================

def _stepper_loop():
    """
    Thread que genera els polsos STEP per a tots els motors.
    Equivalent a task_update_steppers() del FreeRTOS.
    S'inicia automàticament a motion_setup().

    Dues optimitzacions sobre la versió directa, que criden els cinc motors a
    cada volta amb una lectura de rellotge per motor:

      - Els motors ATURATS se salten. Durant el seguiment de línia, els braços
        i la xeringa no es mouen, i les seves crides eren el 69 % del temps del
        bucle (mesurat). Un motor està aturat si no té interval de pas (mode
        velocitat) o si ja és a la seva destinació (mode posició).
      - El rellotge es llegeix UNA vegada per volta i es passa als motors, en
        lloc d'una lectura per motor.

    ⚠️ PROVAT I DESCARTAT (2026-09-11): agrupar els polsos simultanis de les
    dues rodes en una sola crida `set_bank_1()`. No guanya res, perquè estalvia
    una crida però n'afegeix dues (pujar i baixar el nivell) i el sleep del
    pols bloqueja el bucle. Mesurat: 401 passos/s contra 419 sense agrupar.

    Això importa perquè aquest bucle és el que limita la velocitat del robot:
    la cadència de polsos que aconsegueix marca el sostre de MAX_STEP_RATE.
    """
    now_us = Stepper._now_us
    while not _stepper_stop.is_set():
        now = now_us()

        if wheels_speed_mode:
            if wheel_R._step_interval_us:
                wheel_R.run_speed(now)
            if wheel_L._step_interval_us:
                wheel_L.run_speed(now)
        else:
            if wheel_R._target != wheel_R._pos:
                wheel_R.run(now)
            if wheel_L._target != wheel_L._pos:
                wheel_L.run(now)

        if arm_R._target != arm_R._pos:
            arm_R.run(now)
        if arm_L._target != arm_L._pos:
            arm_L.run(now)
        if syringe._target != syringe._pos:
            syringe.run(now)


# ==================
# Tasques (FreeRTOS tasks → funcions bloquejants, cridades des de threads)
# ==================

def task_move_to(expected_target: int, continuar: bool = False):
    """
    Avança una casella. Si la comprovació prèvia detecta un recipient al vèrtex
    següent, s'hi atura al davant; si no, segueix fins a la cruïlla i s'hi
    centra.

    Amb continuar=True es reprèn l'aproximació al recipient des d'on s'havia
    quedat, sense reiniciar el comptador de passos.
    """
    with _Moving():
        global wheels_speed_mode

        global at_object

        speed = 0.0
        hi_ha_objecte = check_object_ahead()   # amb el robot encara aturat
        at_object = hi_ha_objecte
        reset_acceleration()
        wheels_speed_mode = True
        enable_wheels(ON)

        if hi_ha_objecte:
            # S'avança una distància FIXA des del vèrtex on som, seguint la línia
            # perquè el robot hi arribi encarat. No s'espera cap detecció pel camí:
            # el sensor ja ha fet la seva feina abans d'arrencar.
            if not continuar:
                wheels_set_position(0)
            objectiu = mm_to_steps(MM_TO_OBJECT)
            while wheel_L.current_position() < objectiu and not cancelled():
                t_iter = time.monotonic()
                speed = compute_new_speed(speed)
                follow_line_loop(speed, detect_crossing=False, detect_object=False)
                pace_line_follower(t_iter)
            wheels_set_speed(0)
            wheels_speed_mode = False
            enable_wheels(OFF)
            return

        lf_response = CLEAR
        while not cancelled():
            t_iter = time.monotonic()
            speed = compute_new_speed(speed)
            lf_response = follow_line_loop(speed)
            if lf_response != CLEAR:
                break
            pace_line_follower(t_iter)

        if lf_response == CROSSING and expected_target == CROSSING:
            run_to_crossing_center(speed)

        wheels_speed_mode = False
        enable_wheels(OFF)

def task_move_backward(expected_target: int = CROSSING):
    """
    Retrocedeix seguint la línia fins a la cruïlla anterior i s'hi centra.

    Quatre etapes, cadascuna resolent un problema concret:

      1) Retrocedir fins a TROBAR la cinta de la cruïlla on és. En acabar un
         avanç els sensors han quedat passat el vèrtex, sobre blanc.
      2) Retrocedir fins a HAVER-LA CREUAT (les sondes tornen a blanc). Sense
         això, la primera cinta que es trobaria buscant la cruïlla següent
         seria la que acaba de deixar, i el robot no canviaria de casella.
         Es fa per esdeveniment i no per distància, que és més robust.
      3) Seguiment de línia enrere fins a la cruïlla de debò.
      4) Avanç de MM_BACKWARD_TO_CENTER per deixar l'eix sobre el vèrtex, de
         manera que el robot quedi en posició per girar si cal.

    Tot el trajecte enrere va a WHEELS_BACKWARD_SPEED, per sota de la
    velocitat d'anada: follow_line_loop() inverteix el signe de la correcció
    quan forward=False, però el llaç té menys marge en aquest sentit.
    """
    with _Moving():
        global wheels_speed_mode, at_object
        speed = 0.0
        reset_acceleration()
        wheels_speed_mode = True
        enable_wheels(ON)

        def _enrere_fins(cruilla: bool):
            """Retrocedeix fins que les dues sondes vegin negre (o deixin de veure'n)."""
            nonlocal speed
            while not cancelled():
                lf_r, lf_l = _read_line_sensors()
                hi_es = lf_l > BLACK_THRESHOLD and lf_r > BLACK_THRESHOLD
                if hi_es == cruilla:
                    return
                t_iter = time.monotonic()
                speed = min(compute_new_speed(speed), WHEELS_BACKWARD_SPEED)
                follow_line_loop(speed, forward=False)
                pace_line_follower(t_iter)

        if at_object:
            # Venint d'un recipient el robot NO és sobre cap cinta: és enmig de la
            # casella, a MM_TO_OBJECT del vèrtex. La primera cinta que trobi ja és
            # la seva destinació, o sigui que les dues primeres etapes s'ometen.
            # Fer-les el faria creuar el vèrtex d'on ve i anar-se'n al següent.
            _enrere_fins(True)
        else:
            _enrere_fins(True)    # 1) trobar la cinta on som
            _enrere_fins(False)   # 2) acabar de creuar-la
            _enrere_fins(True)    # 3) arribar a la cruïlla anterior
        at_object = False

        # 4) Avançar per centrar l'eix sobre el vèrtex, seguint la línia.
        #    Aquest tram va cap endavant, o sigui que el llaç torna a ser estable:
        #    s'aprofita per realinear el robot amb la línia després del retrocés,
        #    que s'ha fet sense realimentació. Són pocs mil·límetres, però el
        #    control hi treballa a favor en lloc d'estar apagat.
        wheels_set_position(0)
        passos_centre = mm_to_steps(MM_BACKWARD_TO_CENTER)
        speed = WHEELS_MAX_SPEED / 2
        while wheel_L.current_position() < passos_centre and not cancelled():
            t_iter = time.monotonic()
            follow_line_loop(speed, forward=True, detect_crossing=False,
                             detect_object=False)
            pace_line_follower(t_iter)

        wheels_set_speed(0)
        wheels_speed_mode = False
        enable_wheels(OFF)


def task_rotate(direction: bool, continuar: bool = False):
    """
    Gira el robot 90° (CW o CCW).
    Fase 1: acceleració fins al 90% dels passos de rotació.
    Fase 2: desacceleració i ajust fi sobre la línia via sensors analògics.

    Amb continuar=True NO es reinicia el comptador de passos: el gir es reprèn
    des d'on s'havia quedat. És el que cal després d'una aturada a mitges, on
    tornar a començar deixaria el robot girat més del compte.
    """
    with _Moving():
        global wheels_speed_mode, at_object
        at_object = False

        ROTATION_STEPS  = (WHEEL_STEPS_PER_REVOLUTION * ROTATION_PERCENT) // 100
        positive_wheel  = wheel_L if direction == CW else wheel_R

        speed = 0.0
        reset_acceleration()
        wheels_speed_mode = True
        enable_wheels(ON)
        if not continuar:
            wheels_set_position(0)

        while positive_wheel.current_position() < \
                (ROTATION_STEPS * ROTATION_FINE_START) // 100 and not cancelled():
            speed = compute_new_speed(speed)
            wheels_set_speed(speed, rotate=True, direction=direction)
            time.sleep(0.01)

        # Ajust fi: es tanca el gir detectant que un sensor CREUA la cinta
        # perpendicular, és a dir que la veu negra i tot seguit torna a blanc.
        # És una seqüència d'esdeveniments, no una comparació de valors, i per
        # això no és ambigua: la condició anterior ("els dos sensors llegeixen
        # igual") també es compleix amb tots dos sobre blanc lluny de cap cinta,
        # i el gir cap a l'esquerra se'n passava uns 30°.
        #
        # El negre que es veu just començant el gir és el de la línia D'ON VE el
        # robot, no el de la perpendicular, i no s'ha de comptar. Per això la fase
        # a cegues ha de durar prou (ROTATION_FINE_START) per deixar-lo enrere.
        global rotation_ended_on_line
        rotation_ended_on_line = False
        esperant_negre = True
        speed /= 2
        while not cancelled():
            lf_r, lf_l = _read_line_sensors()
            negre = lf_r > BLACK_THRESHOLD or lf_l > BLACK_THRESHOLD

            if esperant_negre:
                if negre:
                    esperant_negre = False
            elif not negre and abs(lf_l - lf_r) < ROTATION_ERROR_THRESHOLD:
                # El sensor ja ha creuat la cinta. Just en sortir-ne encara n'és a
                # tocar i llegeix alt, mentre que l'altre és lluny i llegeix baix.
                # A mesura que el gir continua, el primer s'allunya i el segon
                # s'acosta: les dues lectures es creuen EXACTAMENT quan el robot
                # queda encarat, perquè és quan tots dos són a la mateixa
                # distància de la cinta. Aquest criteri s'autocalibra i no depèn
                # de cap número de passos estimat a ull.
                rotation_ended_on_line = True
                break

            if positive_wheel.current_position() > \
                    (ROTATION_STEPS * ROTATION_FINE_LIMIT) // 100:
                break
            wheels_set_speed(speed, rotate=True, direction=direction)
            time.sleep(0.01)

        # Uns quants passos més per quedar centrat sobre la línia nova.
        if rotation_ended_on_line and ROTATION_FINISH_STEPS > 0:
            fi = positive_wheel.current_position() + ROTATION_FINISH_STEPS
            while positive_wheel.current_position() < fi and not cancelled():
                wheels_set_speed(speed, rotate=True, direction=direction)
                time.sleep(0.01)

        wheels_set_speed(0)
        enable_wheels(OFF)
        wheels_speed_mode = False


def task_take_or_leave_something(take: bool, continuar: bool = False):
    """
    Baixa els braços, opera la xeringa i torna a pujar-los. take=True xucla,
    take=False buida.

    NO es mou de lloc: el robot ja ha d'estar aturat davant del recipient. Fins
    al 2026-09-10 aquesta funció avançava fins a trobar l'objecte, operava i
    tornava sola enrere, tot en una sola ordre. Es va separar perquè amb el
    control remot qui decideix si cal retrocedir o girar és l'usuari, i perquè
    així pot veure què té el robot al davant abans de triar què fa.

    Els braços queden ENERGITZATS en acabar: sense molles no s'aguanten sols i
    han de quedar amunt per no tapar el sensor de gestos.
    """
    with _Moving():
        enable_arms(ON)
        move_arms_to(ARM_LOWER_POSITION)

        # ⚠️ El driver de la xeringa penja d'EN_A, el dels braços (defecte de la
        # PCB): enable_syringe() no és suficient per si sol.
        enable_syringe(ON)
        syringe.move_to(_SY_FULL_EXTENDED_STEPS if take else 0)
        while syringe.distance_to_go() != 0 and not cancelled():
            time.sleep(0.05)
        enable_syringe(OFF)

        move_arms_up()


def task_idle():
    """Pausa breu fins que quibot.py assigni una nova tasca."""
    time.sleep(0.5)
