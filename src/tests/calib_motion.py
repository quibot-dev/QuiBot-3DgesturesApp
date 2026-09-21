"""
calib_motion.py — Banc de calibracio i verificacio del moviment del Qui-Bot H2O.

El test_hardware.py comprova que cada motor i cada sensor responen; aquest fa
MOURE el robot sobre el taulell, que es l'unic que permet mesurar el seu
comportament real i ajustar-ne els parametres. Es l'eina amb que s'han
obtingut els valors del capitol de validacio.

Agrupa tres families de proves:

  Seguiment de linia
    1) Diagnostic estatic — el robot NO es mou; mostra en viu que decidiria el
       seguidor. Serveix per validar BLACK_THRESHOLD movent el robot a ma.
    2) Seguir fins a la 1a cruilla — instrumentat, mesura la frequencia real
       del bucle i registra un CSV.
    3) Recorregut de N cruilles — crida task_move_to(), el cami de produccio.
    4) Ajustar velocitat, P_FACTOR i llindar en viu, sense redesplegar.

  Caracteritzacio dels motors
    5) On van els mil.lisegons (repartiment del temps del bucle)
    6) Prova automatica de velocitats
    7) Velocitat maxima real de pas (demanat vs executat)
    8) Banc de proves: velocitat i acceleracio a ma
    9) Escolta continua, canviant valors sense aturar

  Posicionament
   10) Gir de 90 graus (mesura els passos i si retroba la linia)
   11) Recorregut programat (sequencia d'ordres: A D E R)
   12) Posicions dels bracos
   13) Posicions de la xeringa

Us (a la Raspberry, dins de Rasp/):
    sudo pigpiod -s 1 -x 0xFFFFFFFF
    ~/.venv/bin/python3 tests/calib_motion.py

SEGURETAT: a partir de l'opcio 2 el robot es mou. Tingues espai lliure al
davant i la ma a punt: Ctrl+C atura les rodes immediatament. Les opcions de
recorregut tenen a mes un limit de temps (TIMEOUT_S) perque no se t'escapi.
"""

import os
import sys
import time

# Permet executar des de Rasp/ o des de Rasp/tests/
sys.path.insert(0, "..")
sys.path.insert(0, ".")

import pigpio
import motion
from motion import (
    motion_setup, motion_cleanup,
    enable_wheels, wheels_set_speed,
    follow_line_loop, task_move_to, task_rotate, task_move_backward,
    compute_new_speed, enable_arms, move_arms_to, arms_home, syringe_home,
    task_take_or_leave_something, TAKE, LEAVE, move_arms_up,
    reset_acceleration, distance_to_object,
    CLEAR, CROSSING, OBJECT, ON, OFF, CW, CCW,
)

TIMEOUT_S = 30.0          # limit de seguretat per a les proves en moviment
LOG_DIR   = "logs"


def _connect():
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpiod no esta en marxa. Executa: sudo pigpiod -s 1 -x 0xFFFFFFFF")
        sys.exit(1)
    return pi


def _stop_wheels():
    """Atura les rodes i les desactiva. Segur de cridar diverses vegades."""
    try:
        wheels_set_speed(0)
        motion.wheels_speed_mode = False
        enable_wheels(OFF)
    except Exception:
        pass


def _result_name(res):
    return {CLEAR: "CLEAR", CROSSING: "CRUILLA", OBJECT: "OBJECTE"}.get(res, str(res))


# ==========================================================================
# 1 — Diagnostic estatic (el robot no es mou)
# ==========================================================================

def test_static_diagnostic(pi):
    """
    Replica la decisio de follow_line_loop() SENSE moure les rodes.
    Passa el robot a ma per sobre de la linia i per sobre d'una cruilla i
    comprova que la columna ESTAT digui el que toca a cada lloc.
    """
    print("\n=== DIAGNOSTIC ESTATIC (el robot no es mou) ===")
    print(f"BLACK_THRESHOLD = {motion.BLACK_THRESHOLD}   P_FACTOR = {motion.P_FACTOR}")
    print("\nRecorda: centrat sobre la linia els DOS sensors han de veure BLANC")
    print("(van a banda i banda de la cinta). Els dos negres alhora = CRUILLA.\n")
    print("  dret  esquerre    error  correccio   dist    ESTAT")
    print("  " + "-" * 52)
    try:
        while True:
            r, l = motion._read_line_sensors()
            error = r - l
            corr  = error // motion.P_FACTOR
            mm    = distance_to_object()

            if r > motion.BLACK_THRESHOLD and l > motion.BLACK_THRESHOLD:
                estat = "CRUILLA"
            elif mm < 50:
                estat = "OBJECTE"
            else:
                estat = "linia"

            print(f"  {r:5d}  {l:7d}  {error:+7d}  {corr:+8d}  {mm:5d}    {estat:8s}",
                  end="\r")
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n  (fi)\n")


# ==========================================================================
# 2 — Seguir fins a la primera cruilla, instrumentat
# ==========================================================================

def test_follow_to_crossing(pi):
    """
    Segueix la linia fins a detectar una cruilla, registrant cada iteracio.
    Mateixa estructura que motion.task_move_to(), amb mesura de temps i CSV.
    No centra el robot a la cruilla: aixo es l'opcio 3.
    """
    print("\n=== SEGUIR LINIA FINS A LA 1a CRUILLA ===")
    print(f"BLACK_THRESHOLD = {motion.BLACK_THRESHOLD}   P_FACTOR = {motion.P_FACTOR}")
    if not motion.DETECT_OBJECT:
        print("Deteccio d'objectes DESACTIVADA: nomes s'atura a les cruilles.")
    print(f"Limit de seguretat: {TIMEOUT_S:.0f} s. Ctrl+C per aturar.")
    input("Coloca el robot sobre la linia i prem [Enter] per comencar...")

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime("linia_%Y%m%d_%H%M%S.csv"))
    fh = open(path, "w")
    fh.write("t_s,dret,esquerre,error,correccio,distancia_mm,velocitat,"
             "passos_L,passos_R,resultat\n")

    motion.check_object_ahead()   # amb el robot encara aturat, com fa el robot real
    reset_acceleration()
    speed   = 0.0
    periods = []
    result  = CLEAR
    t0      = time.time()
    t_prev  = t0

    motion.wheels_speed_mode = True
    enable_wheels(ON)
    try:
        while True:
            t_iter = time.monotonic()
            speed  = compute_new_speed(speed)
            result = follow_line_loop(speed)

            now = time.time()
            periods.append(now - t_prev)
            t_prev = now

            r, l  = motion.lf_last_r, motion.lf_last_l
            error = r - l
            # Passos REALMENT executats per cada roda: permet comparar el que
            # el control comanda amb el que el generador de polsos lliura.
            pl = motion.wheel_L.current_position()
            pr = motion.wheel_R.current_position()
            fh.write(f"{now - t0:.4f},{r},{l},{error},{error // motion.P_FACTOR},"
                     f"{motion.lf_last_dist},{speed:.1f},{pl},{pr},"
                     f"{_result_name(result)}\n")

            if result != CLEAR:
                break
            if now - t0 > TIMEOUT_S:
                print(f"\n  TIMEOUT: {TIMEOUT_S:.0f} s sense trobar cap cruilla.")
                break
            motion.pace_line_follower(t_iter)
    except KeyboardInterrupt:
        print("\n  (aturat per l'usuari)")
    finally:
        _stop_wheels()
        fh.close()

    elapsed = time.time() - t0
    print(f"\n  Resultat      : {_result_name(result)}")
    if result == OBJECT:
        print(f"  ATURAT PER OBJECTE a {motion.lf_last_dist} mm (llindar 50 mm).")
        print("  Si no hi havia res al davant, es una lectura falsa del VL53L0X.")
    print(f"  Velocitat final: {speed:.1f} de {motion.WHEELS_MAX_SPEED:.0f} passos/s")
    print(f"  Durada        : {elapsed:.2f} s, {len(periods)} iteracions")
    if periods:
        avg = sum(periods) / len(periods)
        print(f"  Bucle real    : {1.0 / avg:6.1f} Hz  (objectiu "
              f"{motion.LINE_FOLLOWER_FREQ} Hz)")
        print(f"  Periode       : mitja {avg * 1000:.1f} ms, "
              f"max {max(periods) * 1000:.1f} ms")
        if avg > 1.5 * motion.LINE_FOLLOWER_PERIOD:
            print("  NOTA: el bucle va molt mes lent del previst. La causa esperada")
            print("        es la lectura del VL53L0X (30-50 ms) a cada iteracio.")
    print(f"  Registre      : {path}\n")


# ==========================================================================
# 3 — Recorregut de N cruilles (cami de produccio)
# ==========================================================================

def test_run_crossings(pi):
    """
    Encadena N trajectes amb task_move_to(CROSSING), la funcio real que fa
    servir el robot. Cada trajecte acaba centrat sobre la cruilla, aixi que
    aquest test valida tambe MM_TO_CROSSING_CENTER.
    """
    print("\n=== RECORREGUT DE N CRUILLES (task_move_to real) ===")
    try:
        n = int(input("Quantes cruilles? ").strip() or "1")
    except ValueError:
        print("  Numero no valid.")
        return
    print(f"MM_TO_CROSSING_CENTER = {motion.MM_TO_CROSSING_CENTER} mm")
    input("Coloca el robot sobre la linia i prem [Enter] per comencar...")

    try:
        for i in range(1, n + 1):
            print(f"\n  --- cruilla {i}/{n} ---")
            t0 = time.time()
            task_move_to(CROSSING)
            print(f"  Arribat en {time.time() - t0:.2f} s.")
            print("  Mira si el robot ha quedat centrat al vertex.")
            if i < n:
                input("  [Enter] per continuar a la seguent...")
    except KeyboardInterrupt:
        print("\n  (aturat per l'usuari)")
    finally:
        _stop_wheels()
        print()


# ==========================================================================
# 4 — Ajust en viu
# ==========================================================================

def _ask_int(label, valor):
    val = input(f"  {label} [{valor}]: ").strip()
    if not val:
        return valor
    try:
        return int(val)
    except ValueError:
        print("    valor no valid, es deixa igual")
        return valor


def _ask_float(label, valor):
    val = input(f"  {label} [{valor:.0f}]: ").strip()
    if not val:
        return valor
    try:
        return float(val)
    except ValueError:
        print("    valor no valid, es deixa igual")
        return valor


def _ask_bool(label, valor):
    val = input(f"  {label} (s/n) [{'s' if valor else 'n'}]: ").strip().lower()
    return (val == "s") if val in ("s", "n") else valor


def _tune_endavant():
    m = motion
    m.WHEELS_MAX_SPEED = _ask_float("WHEELS_MAX_SPEED (passos/s; 1 mm = 2 passos)",
                                    m.WHEELS_MAX_SPEED)
    m.P_FACTOR         = _ask_int("P_FACTOR (mes GRAN = correccio mes suau)", m.P_FACTOR)
    m.LINE_ERROR_TRIM  = _ask_int("LINE_ERROR_TRIM (mes NEGATIU = tira a l'esquerra)",
                                  m.LINE_ERROR_TRIM)
    m.BLACK_THRESHOLD  = _ask_int("BLACK_THRESHOLD", m.BLACK_THRESHOLD)
    m.DETECT_OBJECT      = _ask_bool("Detectar objectes?", m.DETECT_OBJECT)
    m.MM_TO_OBJECT       = _ask_int("MM_TO_OBJECT (mm des del vertex fins al "
                                    "recipient)", m.MM_TO_OBJECT)
    m.ARM_LOWER_POSITION = _ask_int("ARM_LOWER_POSITION (alcada per xuclar)",
                                    m.ARM_LOWER_POSITION)
    m.SYRINGE_MAX_SPEED  = _ask_float("SYRINGE_MAX_SPEED (passos/s)",
                                      m.SYRINGE_MAX_SPEED)
    motion.syringe.set_max_speed(m.SYRINGE_MAX_SPEED)
    m.OBJECT_AHEAD_MIN_MM = _ask_int("OBJECT_AHEAD_MIN_MM (finestra previa, min)",
                                     m.OBJECT_AHEAD_MIN_MM)
    m.OBJECT_AHEAD_MAX_MM = _ask_int("OBJECT_AHEAD_MAX_MM (finestra previa, max)",
                                     m.OBJECT_AHEAD_MAX_MM)


def _tune_enrere():
    m = motion
    m.WHEELS_BACKWARD_SPEED = _ask_float("WHEELS_BACKWARD_SPEED (positiu; el signe "
                                         "el posa el codi)", m.WHEELS_BACKWARD_SPEED)
    m.BACKWARD_LINE_FOLLOW  = _ask_bool("Seguir la linia amb els sensors?",
                                        m.BACKWARD_LINE_FOLLOW)
    m.BACKWARD_INVERT_CORRECTION = _ask_bool("Invertir el signe de la correccio?",
                                             m.BACKWARD_INVERT_CORRECTION)
    m.P_FACTOR_BACKWARD        = _ask_int("P_FACTOR_BACKWARD (mes GRAN = mes suau)",
                                          m.P_FACTOR_BACKWARD)
    m.LINE_ERROR_TRIM_BACKWARD = _ask_int("LINE_ERROR_TRIM_BACKWARD",
                                          m.LINE_ERROR_TRIM_BACKWARD)
    m.MM_BACKWARD_TO_CENTER    = _ask_int("MM_BACKWARD_TO_CENTER (mm)",
                                          m.MM_BACKWARD_TO_CENTER)


def _tune_cruilla_i_gir():
    m = motion
    m.MM_TO_CROSSING_CENTER   = _ask_int("MM_TO_CROSSING_CENTER (mm)",
                                         m.MM_TO_CROSSING_CENTER)
    m.ROTATION_PERCENT        = _ask_int("ROTATION_PERCENT (% de volta per 90 graus)",
                                         m.ROTATION_PERCENT)
    m.ROTATION_FINE_START     = _ask_int("ROTATION_FINE_START (%)", m.ROTATION_FINE_START)
    m.ROTATION_ERROR_THRESHOLD = _ask_int("ROTATION_ERROR_THRESHOLD",
                                          m.ROTATION_ERROR_THRESHOLD)
    m.ROTATION_FINISH_STEPS   = _ask_int("ROTATION_FINISH_STEPS", m.ROTATION_FINISH_STEPS)


def _mostra_valors():
    m = motion
    print(f"\n  ENDAVANT  velocitat {m.WHEELS_MAX_SPEED:.0f} ({m.WHEELS_MAX_SPEED/2:.0f} mm/s)"
          f" | P_FACTOR {m.P_FACTOR} | trim {m.LINE_ERROR_TRIM}"
          f" | negre {m.BLACK_THRESHOLD} | objectes {'si' if m.DETECT_OBJECT else 'NO'}")
    print(f"  ENRERE    velocitat {m.WHEELS_BACKWARD_SPEED:.0f} | "
          f"sensors {'si' if m.BACKWARD_LINE_FOLLOW else 'NO'} | "
          f"signe {'INVERTIT' if m.BACKWARD_INVERT_CORRECTION else 'normal'} | "
          f"P_FACTOR {m.P_FACTOR_BACKWARD} | trim {m.LINE_ERROR_TRIM_BACKWARD} | "
          f"centrat {m.MM_BACKWARD_TO_CENTER} mm")
    print(f"  CRUILLA   centrat {m.MM_TO_CROSSING_CENTER} mm")
    print(f"  GIR       {m.ROTATION_PERCENT}% | ajust fi des del {m.ROTATION_FINE_START}%"
          f" | llindar {m.ROTATION_ERROR_THRESHOLD} | extra {m.ROTATION_FINISH_STEPS}\n")


def test_tune(pi):
    """Ajust en viu dels parametres, agrupats per blocs."""
    grups = {"1": ("Marxa endavant",   _tune_endavant),
             "2": ("Marxa enrere",     _tune_enrere),
             "3": ("Cruilla i gir",    _tune_cruilla_i_gir)}
    while True:
        print("\n=== AJUST EN VIU ===")
        _mostra_valors()
        for k, (nom, _) in grups.items():
            print(f"    {k}) {nom}")
        print("    0) Tornar al menu")
        opt = input("  Quin bloc? ").strip()
        if opt == "0" or not opt:
            break
        g = grups.get(opt)
        if not g:
            print("  Opcio no valida.")
            continue
        print(f"\n  --- {g[0]} --- (Enter per deixar el valor com esta)")
        g[1]()
    print("  Els canvis valen NOMES per a aquesta sessio. Quan trobis els bons,")
    print("  digue'm els valors i els deixo fixats a motion.py.\n")


def test_timing(pi):
    """
    Mesura quant costa cada lectura de sensor. Diu on se'n va el temps del
    bucle del seguidor. El robot no es mou.
    """
    print("\n=== ON VAN ELS MIL-LISEGONS ===")
    N = 20

    t0 = time.time()
    for _ in range(N):
        motion._read_line_sensors()
    t_line = (time.time() - t0) / N * 1000

    t0 = time.time()
    for _ in range(N):
        distance_to_object()
    t_dist = (time.time() - t0) / N * 1000

    print(f"  Sensors de linia (2 canals ADS1115) : {t_line:6.1f} ms")
    print(f"  Distancia (VL53L0X, bus bit-bang)   : {t_dist:6.1f} ms")

    amortitzat = t_dist / motion.DIST_CHECK_EVERY
    total = t_line + amortitzat + motion.LINE_FOLLOWER_PERIOD * 1000
    print(f"\n  Distancia amortitzada 1 de cada {motion.DIST_CHECK_EVERY}   : {amortitzat:6.1f} ms")
    print(f"  Espera fixa del bucle               : {motion.LINE_FOLLOWER_PERIOD * 1000:6.1f} ms")
    print(f"  ---> iteracio estimada              : {total:6.1f} ms  ({1000 / total:.1f} Hz)")

    mm_per_iter = motion.WHEELS_MAX_SPEED / 2.0 * (total / 1000)
    print(f"\n  A {motion.WHEELS_MAX_SPEED:.0f} passos/s el robot recorre "
          f"{mm_per_iter:.1f} mm entre lectures.")
    print("  Ha de ser forca menys que l'amplada de la cinta (19 mm) per no")
    print("  passar de llarg una cruilla.\n")


SPEED_SWEEP = [130, 200, 300, 400, 500]   # passos/s (1 mm = 2 passos)


def _run_one_leg(max_speed, timeout=TIMEOUT_S):
    """
    Fa un tram fins a la primera cruilla a la velocitat donada.
    Retorna un diccionari amb les mesures. No pregunta res.
    """
    prev_max = motion.WHEELS_MAX_SPEED
    motion.WHEELS_MAX_SPEED = float(max_speed)

    motion.check_object_ahead()
    reset_acceleration()
    speed   = 0.0
    n       = 0
    err_max = 0
    result  = CLEAR
    t0      = time.time()

    motion.wheels_speed_mode = True
    enable_wheels(ON)
    try:
        while True:
            t_iter = time.monotonic()
            speed  = compute_new_speed(speed)
            result = follow_line_loop(speed)
            n += 1
            err_max = max(err_max, abs(motion.lf_last_r - motion.lf_last_l))
            if result != CLEAR:
                break
            if time.time() - t0 > timeout:
                result = None          # temps exhaurit sense trobar cruilla
                break
            motion.pace_line_follower(t_iter)
    finally:
        _stop_wheels()
        motion.WHEELS_MAX_SPEED = prev_max

    elapsed = time.time() - t0
    hz      = n / elapsed if elapsed > 0 else 0.0
    return {
        "speed":   max_speed,
        "mms":     max_speed / 2.0,
        "t":       elapsed,
        "n":       n,
        "hz":      hz,
        "mm_iter": (max_speed / 2.0) / hz if hz > 0 else 0.0,
        "final":   speed,
        "err_max": err_max,
        "result":  result,
    }


def test_speed_sweep(pi):
    """
    Prova la mateixa recta a velocitats creixents i en fa una taula.
    Nomes cal recol.locar el robot i mirar com va: la resta es automatic.
    """
    print("\n=== PROVA DE VELOCITATS ===")
    print("Es fara el mateix tram a cada velocitat, de mes lenta a mes rapida.")
    print("Entre prova i prova torna a posar el robot al mateix lloc.")
    print(f"Velocitats a provar (passos/s): {SPEED_SWEEP}")
    print("Ctrl+C durant un tram el salta; Ctrl+C al menu ho atura tot.\n")

    rows = []
    for sp in SPEED_SWEEP:
        print(f"--- {sp} passos/s  ({sp / 2.0:.0f} mm/s) ---")
        try:
            input("  Coloca el robot sobre la linia i prem [Enter]...")
        except KeyboardInterrupt:
            print("\n  (prova aturada)")
            break
        try:
            row = _run_one_leg(sp)
        except KeyboardInterrupt:
            _stop_wheels()
            print("  (tram saltat)")
            continue

        estat = {CROSSING: "cruilla", OBJECT: "objecte", None: "SENSE CRUILLA"}.get(
            row["result"], "?")
        print(f"  -> {estat} en {row['t']:.2f} s | {row['hz']:.1f} Hz | "
              f"{row['mm_iter']:.1f} mm entre lectures | "
              f"velocitat assolida {row['final']:.0f}/{sp}")
        v = input("  Ha seguit be la linia? (s/n): ").strip().lower()
        row["ok"] = (v == "s")
        rows.append(row)
        print()

    if not rows:
        return

    print("\n================= RESUM =================")
    print(" passos/s   mm/s    Hz   mm/lect   resultat        be?")
    print(" " + "-" * 54)
    for r in rows:
        estat = {CROSSING: "cruilla", OBJECT: "objecte", None: "sense cruilla"}.get(
            r["result"], "?")
        print(f" {r['speed']:7d} {r['mms']:6.0f} {r['hz']:6.1f} {r['mm_iter']:8.1f}"
              f"   {estat:14s} {'si' if r['ok'] else 'NO'}")
    print(" " + "-" * 54)

    bones = [r for r in rows if r["ok"] and r["result"] == CROSSING]
    if bones:
        millor = max(bones, key=lambda r: r["speed"])
        print(f"\n La mes rapida que ha anat be: {millor['speed']:.0f} passos/s "
              f"({millor['mms']:.0f} mm/s).")
        print(" Aquest es el valor que cal fixar a WHEELS_MAX_SPEED (motion.py).")
    else:
        print("\n Cap velocitat ha acabat be. Mira l'opcio 1 abans de continuar.")
    print()


RATE_SWEEP = [130, 200, 300, 500, 800, 1200]   # passos/s a comprovar


def test_max_step_rate(pi):
    """
    Compara els passos DEMANATS amb els REALMENT executats a cada velocitat.
    Separa les dues causes possibles de "no va mes rapid":

      - Si els reals segueixen els demanats i el robot igualment no accelera,
        el limit es MECANIC/ELECTRIC: parell insuficient (Vref dels A4988).
      - Si els reals es queden enrere dels demanats, el limit es de SOFTWARE:
        el bucle de polsos no arriba a generar-los.

    Es mesura la posicio interna dels steppers, o sigui que compta els polsos
    ENVIATS. Fes-ho primer amb les RODES ENLAIRE (sense carrega) per veure el
    sostre del software, i despres a terra per veure el del parell.
    """
    print("\n=== VELOCITAT MAXIMA REAL DE PAS ===")
    print("Recomanat: fes-ho amb el robot AIXECAT, amb les rodes girant lliures.")
    print("Aixi mesures el sostre del software, sense la carrega del terra.")
    try:
        input("[Enter] per comencar (Ctrl+C per sortir)...")
    except KeyboardInterrupt:
        return

    SECONDS = 2.0

    motion.wheels_speed_mode = True
    enable_wheels(ON)

    # Escalfament: la primera velocitat de la tanda havia donat 0 passos una
    # vegada, i no sabem per que. Es fa girar una mica abans de mesurar res,
    # i al final es repeteix la primera velocitat: si llavors surt be, era
    # cosa de l'arrencada i no del valor.
    wheels_set_speed(200.0)
    time.sleep(0.5)
    wheels_set_speed(0)
    time.sleep(0.2)

    print(f"\n  demanat   real   assolit   (mesura de {SECONDS:.0f} s per velocitat)")
    print("  " + "-" * 42)

    try:
        for i, sp in enumerate(RATE_SWEEP + [RATE_SWEEP[0]]):
            repeticio = (i == len(RATE_SWEEP))
            wheels_set_speed(float(sp))
            time.sleep(0.3)                     # deixa que arrenqui
            p0 = motion.wheel_R.current_position()
            t0 = time.time()
            time.sleep(SECONDS)
            dt   = time.time() - t0
            real = abs(motion.wheel_R.current_position() - p0) / dt
            pct  = 100.0 * real / sp
            marca = "" if pct > 90 else "  <-- no hi arriba"
            if repeticio:
                marca += "   (repeticio de la primera)"
            print(f"  {sp:7d} {real:6.0f} {pct:8.0f}%{marca}")
        wheels_set_speed(0)
    except KeyboardInterrupt:
        print("\n  (aturat)")
    finally:
        _stop_wheels()

    print("\n  Si tots surten a prop del 100%, el software genera els polsos de")
    print("  sobres: si el robot no accelera, es parell (puja el Vref dels A4988).")
    print("  Si baixen del 90% a partir d'una velocitat, aquell es el sostre del")
    print("  software i no te sentit demanar-ne mes.\n")


def _ask(label, current, cast=float):
    """Demana un valor i retorna l'actual si es deixa en blanc."""
    val = input(f"  {label} [{current}]: ").strip()
    if not val:
        return current
    try:
        return cast(val)
    except ValueError:
        print("    valor no valid, es deixa igual")
        return current


def test_bench(pi):
    """
    Banc de proves lliure: hi poses velocitat i acceleracio, ho fa girar i
    et diu que ha aconseguit de veritat. Serveix per escoltar el soroll i
    trobar la combinacio que va millor.

    Es fa amb el ROBOT AIXECAT (rodes lliures) o en una recta llarga.
    Repeteix fins que premis 0.
    """
    print("\n=== BANC DE PROVES (velocitat i acceleracio) ===")
    print("Prova els valors que vulguis. Deixa en blanc per no canviar-los.")
    print("Recorda: 1 mm = 2 passos, o sigui 200 passos/s = 100 mm/s.\n")
    print("Sobre el soroll: un motor pas a pas te zones de RESSONANCIA on")
    print("vibra i xiscla mes, sovint a velocitats baixes-mitjanes. Anar mes")
    print("rapid pot sonar MILLOR, no pitjor. Val la pena escoltar tot el rang.\n")

    speed    = motion.WHEELS_MAX_SPEED
    accel    = motion.WHEELS_ACCEL
    duration = 3.0
    ramp     = True

    motion.wheels_speed_mode = True
    try:
        while True:
            print("--- valors ---")
            speed    = _ask("Velocitat (passos/s)", speed)
            accel    = _ask("Acceleracio (passos/s2)", accel)
            duration = _ask("Durada (s)", duration)
            r = input(f"  Amb rampa d'acceleracio? (s/n) [{'s' if ramp else 'n'}]: ").strip().lower()
            if r in ("s", "n"):
                ramp = (r == "s")

            print(f"\n  -> {speed:.0f} passos/s ({speed / 2:.0f} mm/s), "
                  f"accel {accel:.0f}, {duration:.1f} s, "
                  f"{'amb' if ramp else 'sense'} rampa")
            if ramp:
                t_ramp = speed / accel if accel > 0 else 0
                print(f"     temps teoric fins a la velocitat: {t_ramp:.2f} s "
                      f"({t_ramp * speed / 2 / 2:.0f} mm recorreguts accelerant)")
            try:
                input("     [Enter] per arrencar...")
            except KeyboardInterrupt:
                break

            prev_s, prev_a = motion.WHEELS_MAX_SPEED, motion.WHEELS_ACCEL
            motion.WHEELS_MAX_SPEED, motion.WHEELS_ACCEL = speed, accel
            enable_wheels(ON)
            p0 = motion.wheel_R.current_position()
            t0 = time.time()
            try:
                if ramp:
                    reset_acceleration()
                    v = 0.0
                    while time.time() - t0 < duration:
                        v = compute_new_speed(v)
                        wheels_set_speed(v)
                        time.sleep(0.01)
                    reached = v
                else:
                    wheels_set_speed(speed)
                    time.sleep(duration)
                    reached = speed
            except KeyboardInterrupt:
                reached = 0.0
                print("\n     (aturat)")
            finally:
                dt = time.time() - t0
                wheels_set_speed(0)
                enable_wheels(OFF)
                motion.WHEELS_MAX_SPEED, motion.WHEELS_ACCEL = prev_s, prev_a

            passos = abs(motion.wheel_R.current_position() - p0)
            real   = passos / dt if dt > 0 else 0
            print(f"\n     passos executats : {passos} en {dt:.2f} s")
            print(f"     cadencia mitjana : {real:.0f} passos/s ({real / 2:.0f} mm/s)")
            print(f"     ordre final      : {reached:.0f} passos/s")
            if reached > 0 and real < 0.85 * reached:
                print("     NOTA: no ha arribat a la cadencia demanada -> sostre del")
                print("           generador de polsos, demanar-ne mes no serveix.")
            print()

            if input("  Una altra? (Enter per si, 0 per sortir): ").strip() == "0":
                break
    except KeyboardInterrupt:
        pass
    finally:
        _stop_wheels()
        print("\n  Quan trobis els valors bons, digue'ls i els deixo fixats a motion.py.\n")


LISTEN_SPEEDS = [100, 150, 200, 250, 300, 350]      # passos/s
LISTEN_ACCELS = [190, 400, 800, 1600]              # passos/s2


def _parse_list(text, default):
    """Converteix '100 200 300' o '100,200,300' en llista de nombres."""
    text = text.replace(",", " ").strip()
    if not text:
        return default
    try:
        return [float(x) for x in text.split()]
    except ValueError:
        print("    llista no valida, es fa servir la de sempre")
        return default


def _ramp_to(target, accel, tick=0.01):
    """Puja de 0 a target amb l'acceleracio donada. Retorna els segons que ha trigat."""
    prev_a, prev_s = motion.WHEELS_ACCEL, motion.WHEELS_MAX_SPEED
    motion.WHEELS_ACCEL, motion.WHEELS_MAX_SPEED = accel, target
    reset_acceleration()
    v  = 0.0
    t0 = time.time()
    while v < target - 0.5:
        v = compute_new_speed(v)
        wheels_set_speed(v)
        time.sleep(tick)
    wheels_set_speed(target)
    motion.WHEELS_ACCEL, motion.WHEELS_MAX_SPEED = prev_a, prev_s
    return time.time() - t0


def test_listen(pi):
    """
    Escolta continua: les rodes no s'aturen entre valors. Cada [Enter] passa
    al seguent. Aixi el canvi de soroll se sent de cop, sense el silenci del
    mig, que es el que fa dificil comparar.

    Dos modes:
      a) recorre VELOCITATS (sense rampa, salta directament a cada una)
      b) per cada velocitat, primer SENSE rampa i despres amb acceleracions
         creixents, per sentir com sona l'arrencada
    """
    print("\n=== ESCOLTA CONTINUA ===")
    print("Fes-ho amb el robot AIXECAT. Les rodes NO s'aturen entre valors:")
    print("cada [Enter] passa al seguent. Escriu 0 i [Enter] per sortir.\n")

    mode = input("  Mode: (a) velocitats  (b) acceleracions per velocitat [a]: ").strip().lower()
    if mode not in ("a", "b"):
        mode = "a"

    speeds = _parse_list(
        input(f"  Velocitats a provar {LISTEN_SPEEDS}\n  (Enter per aquestes): "),
        LISTEN_SPEEDS)
    accels = LISTEN_ACCELS
    if mode == "b":
        accels = _parse_list(
            input(f"  Acceleracions a provar {LISTEN_ACCELS}\n  (Enter per aquestes): "),
            LISTEN_ACCELS)

    print(f"\n  Sostre mesurat del generador de polsos: ~390 passos/s.")
    print("  Per damunt d'aixo el motor no rep el que li demanes.\n")
    try:
        input("  [Enter] per arrencar...")
    except KeyboardInterrupt:
        return

    motion.wheels_speed_mode = True
    enable_wheels(ON)
    sortir = False
    try:
        if mode == "a":
            for sp in speeds:
                p0 = motion.wheel_R.current_position()
                t0 = time.time()
                wheels_set_speed(float(sp))
                print(f"  --> {sp:.0f} passos/s  ({sp / 2:.0f} mm/s)")
                if input("      [Enter] seguent, 0 per sortir: ").strip() == "0":
                    sortir = True
                real = abs(motion.wheel_R.current_position() - p0) / (time.time() - t0)
                print(f"      real: {real:.0f} passos/s\n")
                if sortir:
                    break
        else:
            for sp in speeds:
                print(f"\n  ===== {sp:.0f} passos/s ({sp / 2:.0f} mm/s) =====")
                wheels_set_speed(0)
                time.sleep(0.4)
                print("  --> SENSE rampa (salt directe)")
                wheels_set_speed(float(sp))
                if input("      [Enter] seguent, 0 per sortir: ").strip() == "0":
                    sortir = True
                if sortir:
                    break
                for ac in accels:
                    wheels_set_speed(0)
                    time.sleep(0.4)
                    t = _ramp_to(float(sp), float(ac))
                    print(f"  --> rampa accel {ac:.0f}: ha trigat {t:.2f} s "
                          f"({t * sp / 2 / 2:.0f} mm accelerant)")
                    if input("      [Enter] seguent, 0 per sortir: ").strip() == "0":
                        sortir = True
                        break
                if sortir:
                    break
        wheels_set_speed(0)
    except KeyboardInterrupt:
        print("\n  (aturat)")
    finally:
        _stop_wheels()

    print("\n  Quins valors t'han sonat millor? Digue'ls i els deixo fixats.\n")


def test_rotate(pi):
    """
    Gir de 90 graus, amb mesura dels passos realment fets.

    task_rotate() te dues fases: primer gira a cegues fins al 90% dels passos
    calculats, i despres ajusta amb els sensors fins a trobar la linia nova o
    fins al 110%. Aquesta prova diu per quina de les dues coses ha acabat: si
    sempre acaba pel limit del 110%, l'ajust fi amb sensors no funciona i el
    gir es completament cec.
    """
    print("\n=== GIR DE 90 GRAUS ===")
    print("Coloca el robot centrat sobre una cruilla, encarat a una linia.")
    d = input("Sentit: (d) dreta / (e) esquerra [d]: ").strip().lower()
    direccio = CCW if d == "e" else CW

    passos = (motion.WHEEL_STEPS_PER_REVOLUTION * motion.ROTATION_PERCENT) // 100
    print(f"\n  ROTATION_PERCENT       : {motion.ROTATION_PERCENT}%")
    print(f"  Passos teorics del gir : {passos}")
    print(f"  Fi de la fase a cegues : {passos * motion.ROTATION_FINE_START // 100}")
    print(f"  Limit de l'ajust fi    : {passos * motion.ROTATION_FINE_LIMIT // 100}")
    try:
        input("\n  [Enter] per girar...")
    except KeyboardInterrupt:
        return

    roda = motion.wheel_L if direccio == CW else motion.wheel_R
    t0 = time.time()
    try:
        task_rotate(direccio)
    except KeyboardInterrupt:
        print("\n  (aturat)")
    finally:
        _stop_wheels()

    fets = abs(roda.current_position())
    print(f"\n  Passos executats : {fets}  ({fets * 100 // passos}% dels teorics)")
    print(f"  Durada           : {time.time() - t0:.2f} s")
    if motion.rotation_ended_on_line:
        print("  --> Ha acabat TROBANT LA LINIA amb els sensors: el gir es tanca")
        print("      sol i el ROTATION_PERCENT nomes marca el punt de partida.")
    else:
        print(f"  --> Ha acabat pel LIMIT del {motion.ROTATION_FINE_LIMIT}% sense trobar")
        print("      la linia. El gir es cec i depen nomes del ROTATION_PERCENT.")
    print("\n  Mira si el robot ha quedat encarat a la linia perpendicular.\n")


def test_route(pi):
    """
    Executa una seqüencia d'ordres d'alt nivell, com les que enviara el
    control remot. Cada ordre es una lletra:

        A  avanca una casella (fins a la cruilla seguent, i s'hi centra)
        D  gira 90 graus a la dreta
        E  gira 90 graus a l'esquerra
        R  retrocedeix una casella
        X  xucla   (baixa bracos, omple la xeringa, puja bracos)
        B  buida   (baixa bracos, buida la xeringa, puja bracos)

    Exemple:  A A D A A E A
    """
    print("\n=== RECORREGUT PROGRAMAT ===")
    print("  A = avanca   D = dreta   E = esquerra   R = enrere")
    print("  X = xucla    B = buida   (el robot ha d'estar davant del recipient)")
    print("  Exemple: A A D A X R     (els espais son opcionals)\n")
    seq = input("  Seqüencia: ").strip().upper().replace(" ", "")
    if not seq:
        return

    accions = {"A": (task_move_to, (CROSSING,), "avanca"),
               "D": (task_rotate,  (CW,),       "gira a la dreta"),
               "E": (task_rotate,  (CCW,),      "gira a l'esquerra"),
               "R": (task_move_backward, (),    "retrocedeix"),
               "X": (task_take_or_leave_something, (TAKE,),  "xucla"),
               "B": (task_take_or_leave_something, (LEAVE,), "buida")}

    desconegudes = [c for c in seq if c not in accions]
    if desconegudes:
        print(f"  Ordres no reconegudes: {' '.join(set(desconegudes))}")
        return

    print(f"\n  {len(seq)} ordres. Coloca el robot sobre una cruilla, encarat")
    print("  a la linia per on ha de comencar. Ctrl+C atura en qualsevol moment.")
    try:
        input("  [Enter] per comencar...")
    except KeyboardInterrupt:
        return

    # Els bracos han d'estar amunt abans de moure's: si van baixos topen amb
    # els recipients i, a mes, tapen el sensor de gestos.
    print("  Pujant els bracos...")
    enable_arms(ON)
    move_arms_up()

    t_total = time.time()
    fetes = 0
    try:
        for i, c in enumerate(seq, 1):
            fn, args, nom = accions[c]
            print(f"  {i}/{len(seq)}  {c} — {nom}...", end="", flush=True)
            t0 = time.time()
            fn(*args)
            print(f" fet en {time.time() - t0:.1f} s")
            fetes += 1
            time.sleep(0.3)          # petita pausa entre ordres
    except KeyboardInterrupt:
        print("\n  (aturat per l'usuari)")
    finally:
        _stop_wheels()

    print(f"\n  {fetes}/{len(seq)} ordres executades en "
          f"{time.time() - t_total:.1f} s")
    print("  Mira on ha quedat el robot respecte d'on hauria de ser.\n")


def test_arms(pi):
    """
    Mou els bracos a passes per trobar les posicions limit.

    Els bracos NO es queden quiets sols: les molles no aguanten la posicio
    alta, i per tant els motors han d'estar energitzats sempre. Aquesta prova
    els deixa habilitats en sortir, perque si no cauen i es perd la posicio.
    """
    print("\n=== POSICIONS DELS BRACOS ===")
    print("  Els bracos queden energitzats: si es desactiven, cauen.")
    print(f"  Constants actuals: baix {motion.ARM_LOWER_POSITION}, "
          f"dalt esquerre {motion.ARM_L_UPPER_POSITION}, "
          f"dalt dret {motion.ARM_R_UPPER_POSITION}\n")
    print("  Ordres:")
    print("     50 / -50   mou els DOS bracos aquest nombre de passos")
    print("     l50 / l-50 nomes l'esquerre")
    print("     r50 / r-50 nomes el dret")
    print("     z          posa la posicio actual com a ZERO (baix del tot)")
    print("     h          fa el homing (baixa a cegues i puja)")
    print("     q          sortir\n")

    enable_arms(ON)
    try:
        while True:
            pl = motion.arm_L.current_position()
            pr = motion.arm_R.current_position()
            cmd = input(f"  [L={pl}  R={pr}] > ").strip().lower()
            if cmd in ("q", ""):
                break
            if cmd == "z":
                motion.arm_L.set_current_position(0)
                motion.arm_R.set_current_position(0)
                print("     zero fixat")
                continue
            if cmd == "h":
                print("     homing... (baixa a cegues, compte amb el topall)")
                arms_home()
                continue
            try:
                if cmd[0] == "l":
                    motion.arm_L.move(int(cmd[1:]))
                elif cmd[0] == "r":
                    motion.arm_R.move(int(cmd[1:]))
                else:
                    n = int(cmd)
                    motion.arm_L.move(n)
                    motion.arm_R.move(n)
            except ValueError:
                print("     ordre no valida")
                continue
            while motion.arm_L.distance_to_go() or motion.arm_R.distance_to_go():
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  (aturat)")

    print(f"\n  Posicio final: L={motion.arm_L.current_position()}  "
          f"R={motion.arm_R.current_position()}")
    print("  Els bracos queden habilitats. Digue'm els numeros i els fixo.\n")


def test_syringe(pi):
    """
    Mou la xeringa a passes per trobar el recorregut real.

    NOTA: el driver de la xeringa penja d'EN_A, el dels bracos, no d'EN_SERVO.
    Es un defecte conegut de la PCB: per moure la xeringa cal habilitar els
    bracos, i tots dos queden energitzats alhora.
    """
    print("\n=== POSICIONS DE LA XERINGA ===")
    print("  Compte: en habilitar la xeringa s'habiliten tambe els bracos")
    print("  (comparteixen l'ENABLE a la placa).")
    print(f"  Constants actuals: recorregut complet "
          f"{motion._SY_FULL_EXTENDED_STEPS} passos, "
          f"homing {motion.SYRINGE_HOMING_STEPS}\n")
    print("  Ordres:")
    print("     500 / -500  mou la xeringa aquest nombre de passos")
    print("     z           posa la posicio actual com a ZERO")
    print("     h           fa el homing")
    print("     q           sortir\n")

    enable_arms(ON)
    try:
        while True:
            cmd = input(f"  [posicio = {motion.syringe.current_position()}] > ").strip().lower()
            if cmd in ("q", ""):
                break
            if cmd == "z":
                motion.syringe.set_current_position(0)
                print("     zero fixat")
                continue
            if cmd == "h":
                print("     homing...")
                syringe_home()
                enable_arms(ON)
                continue
            try:
                motion.syringe.move(int(cmd))
            except ValueError:
                print("     ordre no valida")
                continue
            while motion.syringe.distance_to_go():
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  (aturat)")

    print(f"\n  Posicio final: {motion.syringe.current_position()}")
    print("  Digue'm el recorregut complet i el punt de repos i els fixo.\n")


MENU = """
===== BANC DE CALIBRACIO DEL MOVIMENT — QUI-BOT H2O =====
    1) Diagnostic estatic (el robot NO es mou)
    2) Seguir fins a la 1a cruilla (instrumentat + CSV)
    3) Recorregut de N cruilles (task_move_to real)
    4) Ajustar velocitat / P_FACTOR / llindar en viu
    5) On van els mil-lisegons (mesura de temps)
    6) PROVA DE VELOCITATS (automatica, fa la taula ella sola)
    7) Velocitat maxima real de pas (demanat vs executat)
    8) BANC DE PROVES: hi poses velocitat i acceleracio i escoltes
    9) ESCOLTA CONTINUA: no s'atura, [Enter] canvia de valor
   10) GIR DE 90 GRAUS (mesura els passos i si troba la linia)
   11) RECORREGUT PROGRAMAT (seqüencia d'ordres: A D E R)
   12) POSICIONS DELS BRACOS (moure a passes i llegir)
   13) POSICIONS DE LA XERINGA
    0) Sortir
======================================================"""


def main():
    pi = _connect()
    print("Inicialitzant motors i sensors (motion_setup)...")
    motion_setup(pi)
    enable_wheels(OFF)

    tests = {
        "1": test_static_diagnostic,
        "2": test_follow_to_crossing,
        "3": test_run_crossings,
        "4": test_tune,
        "5": test_timing,
        "6": test_speed_sweep,
        "7": test_max_step_rate,
        "8": test_bench,
        "9": test_listen,
        "10": test_rotate,
        "11": test_route,
        "12": test_arms,
        "13": test_syringe,
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
                finally:
                    _stop_wheels()
            else:
                print("Opcio no valida.")
    finally:
        print("Aturant i desactivant motors...")
        _stop_wheels()
        motion_cleanup()
        time.sleep(0.2)
        pi.stop()
        print("Fet.")


if __name__ == "__main__":
    main()
