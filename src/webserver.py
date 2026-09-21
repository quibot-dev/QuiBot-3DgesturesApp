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
webserver.py — Control remot del Qui-Bot H2O per Wi-Fi.

Serveix una pagina web i una API REST. NO es un programa independent: el
llanca `quibot.py` en un thread, de manera que comparteix estat i maquinari
amb els modes de gestos i de blocs.

Jerarquia de control:

    gestos  i  blocs      mateix nivell; un gest canvia d'un a l'altre
    ------------------------------------------------------------------
    web                   per sobre: pot aturar una accio en curs,
                          canviar de mode, calibrar i apagar el robot

Les accions de moviment passen pel mateix `_execute_action()` que fan servir
gestos i blocs, aixi que s'encuen i mai es trepitgen. L'aturada d'emergencia,
en canvi, NO fa cua: ha d'interrompre.

Us:  el robot s'engega amb `sudo python3 quibot.py` i la web queda a
     http://<ip-del-robot>:8000
"""

import os
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

import motion
from motion import (
    task_move_to, task_move_backward, task_rotate,
    task_take_or_leave_something,
    arms_home, syringe_home,
    enable_wheels, enable_arms, enable_syringe, wheels_set_speed,
    CROSSING, TAKE, LEAVE, CW, CCW, ON, OFF,
)

# Límit de passos per moviment manual, perquè una petició no pugui manar un
# recorregut absurd.
_MAX_JOG = 2000

PORT = 8000

# Referencies al programa principal, injectades des de quibot.py per no crear
# una dependencia circular entre els dos moduls.
_robot = None


def bind(robot):
    """
    Rep l'objecte que dona acces a l'estat del programa principal. Ha de tenir:
        execute_action(fn, *args)   executa una accio amb el lock compartit
        is_busy()                   hi ha una accio en curs?
        get_mode() / set_mode(g)    mode gestos (True) o blocs (False)
        request_shutdown()          atura el programa
    """
    global _robot
    _robot = robot


# ==========================================================================
# Estat
# ==========================================================================

_last_action = {"name": "cap", "at": 0.0, "ok": True}

# Acció que ha quedat a mitges per una aturada. Es guarda per poder-la repetir,
# però MAI es reprèn sola: després d'aturar, el robot queda enmig d'una casella
# i pot ser que algú l'hagi mogut de lloc. Qui decideix és l'usuari.
_interrompuda = None


def _run(name, fn, *args, ruta=None, ordre=None, continuar=False):
    """
    Executa una accio de moviment amb el lock compartit. Si ja n'hi ha una en
    curs respon 409 en lloc d'encuar-se indefinidament: qui fa servir la web
    ha de saber que el robot esta ocupat, no quedar-se esperant.
    """
    if _robot.is_busy():
        return jsonify({"error": "El robot esta ocupat",
                        "busy": True}), 409

    def feina():
        global _last_action, _interrompuda
        _interrompuda = None
        try:
            if continuar:
                _robot.execute_action(fn, *args, continuar=True)
            else:
                _robot.execute_action(fn, *args)
            if motion.cancelled() and ordre:
                _interrompuda = {"nom": name, "ruta": ruta, "ordre": ordre}
                _last_action = {"name": name + " (aturada)",
                                "at": time.time(), "ok": False}
            else:
                _last_action = {"name": name, "at": time.time(), "ok": True}
        except Exception as e:                       # noqa: BLE001
            _last_action = {"name": name, "at": time.time(),
                            "ok": False, "error": str(e)}

    threading.Thread(target=feina, daemon=True, name=f"web-{name}").start()
    return jsonify({"action": name, "accepted": True})


# ==========================================================================
# Aplicacio
# ==========================================================================

app = Flask(__name__, static_folder=None)


@app.after_request
def _no_cache(resp):
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/")
def index():
    return send_from_directory(os.path.join(os.path.dirname(__file__), "web"),
                               "index.html")


@app.route("/static/<path:nom>")
def static_files(nom):
    return send_from_directory(os.path.join(os.path.dirname(__file__), "web"),
                               nom)


_ACCIONS = {
    "A": (task_move_to, (CROSSING,), "avança"),
    "D": (task_rotate, (CW,), "gira a la dreta"),
    "E": (task_rotate, (CCW,), "gira a l'esquerra"),
    "R": (task_move_backward, (), "retrocedeix"),
    "X": (task_take_or_leave_something, (TAKE,), "xucla"),
    "B": (task_take_or_leave_something, (LEAVE,), "buida"),
}

# ---------- moviment ----------

@app.post("/api/move/forward")
def move_forward():
    return _run("endavant", task_move_to, CROSSING, ruta="move/forward", ordre="A")


@app.post("/api/move/backward")
def move_backward():
    return _run("enrere", task_move_backward, ruta="move/backward", ordre="R")


@app.post("/api/move/left")
def move_left():
    return _run("esquerra", task_rotate, CCW, ruta="move/left", ordre="E")


@app.post("/api/move/right")
def move_right():
    return _run("dreta", task_rotate, CW, ruta="move/right", ordre="D")


# ---------- xeringa ----------

@app.post("/api/syringe/take")
def syringe_take():
    return _run("xuclar", task_take_or_leave_something, TAKE, ruta="syringe/take", ordre="X")


@app.post("/api/syringe/leave")
def syringe_leave():
    return _run("buidar", task_take_or_leave_something, LEAVE, ruta="syringe/leave", ordre="B")


# ---------- aturada ----------

@app.post("/api/stop")
def stop():
    """
    Aturada immediata. NO passa pel lock d'accions: ha de poder interrompre
    una accio en curs, que es justament qui te el lock agafat.
    """
    # Primer de tot, demanar a l'acció en curs que s'aturi. Sense això les
    # rodes es paraven però el bucle de l'acció seguia esperant una cruïlla que
    # ja no arribaria mai: mantenia el lock i el robot quedava bloquejat.
    motion.cancel_action()
    _seq["aturar"] = True

    # Cada pas va en el seu try: l'aturada ha de fer tot el que pugui encara
    # que alguna cosa falli. És el botó que no es pot permetre petar.
    for accio in (lambda: motion.wheels_set_speed(0),
                  lambda: motion.wheel_L.stop(),
                  lambda: motion.wheel_R.stop(),
                  lambda: enable_wheels(OFF)):
        try:
            accio()
        except Exception:                                   # noqa: BLE001
            pass
    return jsonify({"action": "aturat"})


@app.post("/api/poweroff")
def poweroff():
    """Atura el programa i apaga la Raspberry."""
    _robot.request_shutdown()
    threading.Timer(2.0, lambda: os.system("sudo poweroff")).start()
    return jsonify({"action": "apagant"})


@app.post("/api/resume")
def resume():
    """
    CONTINUA l'acció que va quedar interrompuda, des d'on s'havia quedat.

    Les accions passen `continuar=True`, que fa que NO reiniciïn el comptador
    de passos. És important sobretot al gir: tornar a començar un gir ja fet a
    mitges deixaria el robot girat més del compte.
    """
    if _interrompuda is None:
        return jsonify({"error": "No hi ha cap acció interrompuda"}), 400
    fn, args, nom = _ACCIONS[_interrompuda["ordre"]]
    return _run(nom, fn, *args, ruta=_interrompuda["ruta"],
                ordre=_interrompuda["ordre"], continuar=True)


# ---------- mode ----------

@app.post("/api/mode")
def set_mode():
    dades = request.get_json(silent=True) or {}
    mode = str(dades.get("mode", request.values.get("mode", ""))).lower()
    if mode not in ("gestos", "blocs"):
        return jsonify({"error": "mode ha de ser 'gestos' o 'blocs'"}), 400
    _robot.set_mode(mode == "gestos")
    return jsonify({"mode": mode})


# ---------- calibratge (mode avancat) ----------

@app.post("/api/calibrate/arms")
def calibrate_arms():
    return _run("calibrar braços", arms_home)


@app.post("/api/calibrate/syringe")
def calibrate_syringe():
    return _run("calibrar xeringa", syringe_home)


# ---------- moviment manual (mode avancat) ----------

def _jog(motors, passos, nom):
    """Mou un motor un nombre de passos relatiu, amb el lock compartit."""
    try:
        passos = int(passos)
    except (TypeError, ValueError):
        return jsonify({"error": "passos ha de ser un nombre"}), 400
    passos = max(-_MAX_JOG, min(_MAX_JOG, passos))

    def feina():
        enable_arms(ON)          # el driver de la xeringa penja d'EN_A
        for m in motors:
            m.move(passos)
        while any(m.distance_to_go() for m in motors):
            time.sleep(0.02)

    return _run(f"{nom} {passos:+d}", feina)


@app.post("/api/jog/arms")
def jog_arms():
    dades = request.get_json(silent=True) or request.values
    return _jog([motion.arm_L, motion.arm_R], dades.get("passos", 0), "braços")


@app.post("/api/jog/syringe")
def jog_syringe():
    dades = request.get_json(silent=True) or request.values
    return _jog([motion.syringe], dades.get("passos", 0), "xeringa")


@app.post("/api/jog/zero")
def jog_zero():
    """Fixa la posicio actual com a zero, sense moure res."""
    quin = str((request.get_json(silent=True) or request.values).get("quin", ""))
    if quin == "arms":
        motion.arm_L.set_current_position(0)
        motion.arm_R.set_current_position(0)
    elif quin == "syringe":
        motion.syringe.set_current_position(0)
    else:
        return jsonify({"error": "quin ha de ser 'arms' o 'syringe'"}), 400
    return jsonify({"zero": quin})


# ---------- sequencies ----------


_seq = {"passos": [], "index": 0, "executant": False, "aturar": False}


@app.post("/api/sequence")
def sequence():
    """
    Executa una seqüencia d'ordres, una darrere l'altra. Cada ordre passa pel
    lock compartit, o sigui que gestos i blocs poden intercalar-s'hi si el
    robot esta en aquell mode. L'aturada la cancel.la.
    """
    if _seq["executant"]:
        return jsonify({"error": "Ja hi ha una seqüencia en marxa"}), 409
    if _robot.is_busy():
        return jsonify({"error": "El robot esta ocupat"}), 409

    text = str((request.get_json(silent=True) or request.values).get("ordres", ""))
    ordres = [c for c in text.upper() if not c.isspace()]
    desconegudes = sorted({c for c in ordres if c not in _ACCIONS})
    if desconegudes:
        return jsonify({"error": "Ordres no reconegudes: " + " ".join(desconegudes)}), 400
    if not ordres:
        return jsonify({"error": "Seqüencia buida"}), 400

    _seq.update({"passos": ordres, "index": 0, "executant": True, "aturar": False})

    def feina():
        global _last_action
        try:
            for i, c in enumerate(ordres):
                if _seq["aturar"]:
                    break
                _seq["index"] = i
                fn, args, nom = _ACCIONS[c]
                _robot.execute_action(fn, *args)
                _last_action = {"name": nom, "at": time.time(), "ok": True}
                time.sleep(0.3)
        except Exception as e:                              # noqa: BLE001
            _last_action = {"name": "seqüencia", "at": time.time(),
                            "ok": False, "error": str(e)}
        finally:
            _seq["executant"] = False

    threading.Thread(target=feina, daemon=True, name="web-seq").start()
    return jsonify({"ordres": ordres, "accepted": True})


@app.post("/api/sequence/stop")
def sequence_stop():
    _seq["aturar"] = True
    return jsonify({"aturant": True})


# ---------- estat ----------

@app.get("/api/status")
def status():
    """Estat del robot. La pagina el consulta un cop per segon."""
    return jsonify({
        "ocupat":      _robot.is_busy(),
        "mode":        "gestos" if _robot.get_mode() else "blocs",
        "recipient":   motion._object_expected,
        "davant_recipient": motion.at_object,
        "distancia_mm": motion.lf_last_dist,
        "bracos":      motion.arm_L.current_position() if motion.arm_L else 0,
        "xeringa":     motion.syringe.current_position() if motion.syringe else 0,
        "xeringa_max": motion._SY_FULL_EXTENDED_STEPS,
        "ultima_accio": _last_action,
        "interrompuda": _interrompuda["nom"] if _interrompuda else None,
        "sequencia": {
            "executant": _seq["executant"],
            "index":     _seq["index"],
            "total":     len(_seq["passos"]),
            "ordres":    _seq["passos"],
        },
    })


@app.errorhandler(404)
def _404(e):
    return jsonify({"error": "No trobat"}), 404


@app.errorhandler(500)
def _500(e):
    return jsonify({"error": "Error intern"}), 500


def run():
    """Arrenca el servidor. Cridat des d'un thread de quibot.py."""
    app.run(host="0.0.0.0", port=PORT, debug=False,
            use_reloader=False, threaded=True)
