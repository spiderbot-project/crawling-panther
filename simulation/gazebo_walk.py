"""Gazebo'daki robotu PyBullet'te bulunan yürüyüşle (salınım 15°, kaldırma 55°) yürütür.
Önce başka bir terminalde:  gz sim -r simulation/robot_world_v2.sdf
Sonra:                      python3 gazebo_walk.py
Yön düzeltmesi yok (açık döngü), sapma beklenir.
"""
import importlib
import math
import subprocess
import time

import sys

# kullanım: python3 gazebo_walk.py [salınım_derece] [kaldırma_derece]
HIP_SWING_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 10
KNEE_LIFT_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 30
print(f"Yürüyüş: salınım {HIP_SWING_DEG}°, kaldırma {KNEE_LIFT_DEG}°")
INTERP_STEPS = 40
STEP_DT = 1.0 / 240.0

TOPIC_OF = {
    "R1": "/cmd/hip1", "R2": "/cmd/hip2", "R3": "/cmd/leg1", "R4": "/cmd/leg2",
    "L1": "/cmd/hip3", "L2": "/cmd/hip4", "L3": "/cmd/leg3", "L4": "/cmd/leg4",
}
HIP_FORWARD_SIGN = {"R1": -1, "R2": -1, "L1": +1, "L2": +1}
KNEE_SIGN = {"R3": +1, "R4": +1, "L3": -1, "L4": -1}   # PyBullet'te bulunan "içeri kalkış"


def _find_gz_python():
    """gz-transport / gz-msgs Python bağlarını sürüm numarası aramadan bulmaya çalışır."""
    node_cls = double_cls = None
    for v in range(20, 5, -1):
        try:
            node_cls = importlib.import_module(f"gz.transport{v}").Node
            break
        except Exception:
            continue
    for v in range(20, 5, -1):
        try:
            double_cls = importlib.import_module(f"gz.msgs{v}.double_pb2").Double
            break
        except Exception:
            continue
    return node_cls, double_cls


NodeCls, DoubleCls = _find_gz_python()
if NodeCls and DoubleCls:
    node = NodeCls()
    pubs = {k: node.advertise(t, DoubleCls) for k, t in TOPIC_OF.items()}
    print("Python gz bağları kullanılıyor (hızlı).")

    def send(cmd):
        for k, v in cmd.items():
            m = DoubleCls()
            m.data = v
            pubs[k].publish(m)
else:
    print("Python gz bağları bulunamadı, `gz topic` komutuna düşülüyor (yavaş).")

    def send(cmd):
        procs = [subprocess.Popen(
            ["gz", "topic", "-t", TOPIC_OF[k], "-m", "gz.msgs.Double", "-p", f"data: {v:.4f}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for k, v in cmd.items()]
        for pr in procs:
            pr.wait()

current = {k: 0.0 for k in TOPIC_OF}
target = {k: 0.0 for k in TOPIC_OF}


def move():
    if not (NodeCls and DoubleCls):
        # yavaş yol: hedefi bir kez gönder, ara noktaları Gazebo'nun PD kontrolcüsü yumuşatsın
        for k in current:
            current[k] = target[k]
        send(current)
        time.sleep(INTERP_STEPS * STEP_DT)
        return
    start = dict(current)
    for s in range(1, INTERP_STEPS + 1):
        for k in current:
            current[k] = start[k] + (target[k] - start[k]) * s / INTERP_STEPS
        send(current)
        time.sleep(STEP_DT)


def hip(k, d):
    target[k] = HIP_FORWARD_SIGN[k] * math.radians(HIP_SWING_DEG) * d


def knee(k, up):
    target[k] = KNEE_SIGN[k] * math.radians(KNEE_LIFT_DEG) if up else 0.0


def gait_step():
    knee("R3", True); knee("L4", True)
    hip("R1", +1); hip("L2", +1); hip("R2", -1); hip("L1", -1); move()
    knee("R3", False); knee("L4", False); move()
    knee("R4", True); knee("L3", True)
    hip("R2", +1); hip("L1", +1); hip("R1", -1); hip("L2", -1); move()
    knee("R4", False); knee("L3", False); move()


if __name__ == "__main__":
    print("2 sn bekleniyor (robot otursun), sonra yürüyüş. Durdurmak için Ctrl+C.")
    send(current)
    time.sleep(2)
    n = 0
    try:
        while True:
            gait_step()
            n += 1
            if n % 5 == 0:
                print("döngü", n)
    except KeyboardInterrupt:
        for k in target:
            target[k] = 0.0
        move()
        print("Durduruldu, robot dinlenme duruşunda.")
