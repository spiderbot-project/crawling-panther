"""Learns the robot's walking gait by itself (evolutionary search, no extra libraries).

How it works:
  1) Each candidate = 6 numbers: right/left hip swing, right/left knee lift,
     movement speed (interpolation steps) and heading-correction gain (kp).
  2) Each candidate walks 40 cycles in PyBullet and gets a score:
       score = forward distance - sideways drift - turning - fall penalty
  3) The best candidates are mutated into a new generation; repeated for 25 generations.
  4) The best gait is written to best_gait_v2.json (next to this script).

Default bounds are realistic for the real robot (hip swing <= 25 deg, knee lift <= 45 deg).
Add --serbest for the wider bounds (hip <= 40 deg, knee <= 70 deg).

Usage (run from the simulation/ folder):
  python learn_gait.py            # learn (no window, a few minutes)
  python learn_gait.py --izle     # watch the gait saved in best_gait_v2.json in a window
"""
import json
import math
import os
import random
import sys
import time

import pybullet as p
import pybullet_data

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "crawling-panther_v2.urdf")
BEST_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_gait_v2.json")
START_Z = 0.03
CYCLES = 40          # 15 döngülük sınav aşırı uyuma yol açtı, dönüşün birikmesini görmek için uzun
MAX_FORCE = 0.25
MAX_VELOCITY = 8.0

JOINT_OF = {
    "R1": "base_link_to_hip1", "R2": "base_link_to_hip2",
    "R3": "base_link_to_leg1", "R4": "base_link_to_leg2",
    "L1": "base_link_to_hip3", "L2": "base_link_to_hip4",
    "L3": "base_link_to_leg3", "L4": "base_link_to_leg4",
}
HIP_SIGN = {"R1": -1, "R2": -1, "L1": +1, "L2": +1}
KNEE_SIGN = {"R3": +1, "R4": +1, "L3": -1, "L4": -1}

# aday parametreleri: (alt sınır, üst sınır)
# varsayılan: gerçek robota uygun sınırlar (salınım büyüyünce bacaklar çarpışıyor, MG90S diz için sınırlı)
BOUNDS = {
    "swingR": (5, 25), "swingL": (5, 25),     # kalça salınımı (derece)
    "liftR": (20, 45), "liftL": (20, 45),     # diz kaldırma (derece)
    "steps": (15, 60),                        # bir hareketin ara adım sayısı (düşük = hızlı)
    "kp": (0.0, 1.5),                         # yön düzeltme kazancı (jiroskop mantığı)
}
if "--serbest" in sys.argv:                   # eski geniş sınırlar
    BOUNDS.update({"swingR": (5, 40), "swingL": (5, 40), "liftR": (20, 70), "liftL": (20, 70)})
KEYS = list(BOUNDS)


def clamp(params):
    return {k: min(BOUNDS[k][1], max(BOUNDS[k][0], params[k])) for k in KEYS}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def simulate(params, gui=False, cycles=CYCLES, verbose=False):
    cid = p.connect(p.GUI if gui else p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        plane = p.loadURDF("plane.urdf")
        p.changeDynamics(plane, -1, lateralFriction=2.5, spinningFriction=0.5, rollingFriction=0.1, restitution=0.0)
        rid = p.loadURDF(URDF_PATH, [0, 0, START_Z], p.getQuaternionFromEuler([0, 0, 0]))
        idx = {}
        for i in range(p.getNumJoints(rid)):
            idx[p.getJointInfo(rid, i)[1].decode()] = i
            p.changeDynamics(rid, i, lateralFriction=0.5, spinningFriction=0.2, restitution=0.0,
                             linearDamping=0.05, angularDamping=0.05)
        swing = {"R": params["swingR"], "L": params["swingL"]}
        kp = params.get("kp", 0.0)
        scale = {"R": 1.0, "L": 1.0}
        lift = {"R": params["liftR"], "L": params["liftL"]}
        steps = int(round(params["steps"]))
        cur = {k: 0.0 for k in JOINT_OF}
        tgt = {k: 0.0 for k in JOINT_OF}

        def move():
            s0 = dict(cur)
            for s in range(1, steps + 1):
                for k, jn in JOINT_OF.items():
                    cur[k] = s0[k] + (tgt[k] - s0[k]) * s / steps
                    p.setJointMotorControl2(rid, idx[jn], p.POSITION_CONTROL, targetPosition=cur[k],
                                            force=MAX_FORCE, maxVelocity=MAX_VELOCITY)
                p.stepSimulation()
                if gui:
                    time.sleep(1 / 240)

        def hip(k, d):
            tgt[k] = HIP_SIGN[k] * math.radians(swing[k[0]]) * scale[k[0]] * d

        def knee(k, up):
            tgt[k] = KNEE_SIGN[k] * math.radians(lift[k[0]]) if up else 0.0

        for _ in range(170):
            p.stepSimulation()
            if gui:
                time.sleep(1 / 240)
        pos0, o0 = p.getBasePositionAndOrientation(rid)
        yaw0 = p.getEulerFromQuaternion(o0)[2]
        worst_tilt = 0.0
        for c in range(cycles):
            yaw_now = wrap(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(rid)[1])[2] - yaw0)
            steer = kp * (-yaw_now)
            scale["R"] = min(1.8, max(0.2, 1.0 - steer))
            scale["L"] = min(1.8, max(0.2, 1.0 + steer))
            knee("R3", True); knee("L4", True)
            hip("R1", +1); hip("L2", +1); hip("R2", -1); hip("L1", -1); move()
            knee("R3", False); knee("L4", False); move()
            knee("R4", True); knee("L3", True)
            hip("R2", +1); hip("L1", +1); hip("R1", -1); hip("L2", -1); move()
            knee("R4", False); knee("L3", False); move()
            pos, o = p.getBasePositionAndOrientation(rid)
            r, pch, y = p.getEulerFromQuaternion(o)
            worst_tilt = max(worst_tilt, abs(math.degrees(r)), abs(math.degrees(pch)))
            if verbose and (c + 1) % 5 == 0:
                print(f"döngü {c+1:3d} | ileri={100*(pos[0]-pos0[0]):+7.1f} cm  yan={100*(pos[1]-pos0[1]):+7.1f} cm"
                      f"  yön={math.degrees(wrap(y-yaw0)):+6.1f}°  eğim={worst_tilt:4.1f}°")
        pos1, o1 = p.getBasePositionAndOrientation(rid)
        dx = (pos1[0] - pos0[0]) * 100
        dy = (pos1[1] - pos0[1]) * 100
        dyaw = abs(math.degrees(wrap(p.getEulerFromQuaternion(o1)[2] - yaw0)))
        score = dx - abs(dy) - 0.3 * dyaw - (60 if worst_tilt > 35 else 0)
        return score, dx, dy, dyaw, worst_tilt
    finally:
        p.disconnect()


def learn(generations=25, pop=20, elite=5):
    random.seed(1)
    center = {"swingR": 15, "swingL": 15, "liftR": 35, "liftL": 35, "steps": 40, "kp": 0.3}
    sigma = {k: 0.25 * (BOUNDS[k][1] - BOUNDS[k][0]) for k in KEYS}
    best_all = (-1e9, None, None)
    for g in range(generations):
        cands = [clamp(center)] + [
            clamp({k: random.gauss(center[k], sigma[k]) for k in KEYS}) for _ in range(pop - 1)]
        scored = []
        for c in cands:
            res = simulate(c)
            scored.append((res[0], c, res))
        scored.sort(key=lambda t: t[0], reverse=True)
        if scored[0][0] > best_all[0]:
            best_all = scored[0]
        top = scored[:elite]
        center = {k: sum(t[1][k] for t in top) / elite for k in KEYS}
        sigma = {k: max(0.03 * (BOUNDS[k][1] - BOUNDS[k][0]), sigma[k] * 0.88) for k in KEYS}
        s, c, r = scored[0]
        print(f"nesil {g+1:2d}/{generations} | en iyi skor {s:+7.1f} | ileri {r[1]:+6.1f} cm  yan {r[2]:+6.1f} cm"
              f"  dönüş {r[3]:5.1f}°  eğim {r[4]:4.1f}° | " + " ".join(f"{k}={c[k]:.0f}" for k in KEYS), flush=True)
    score, params, res = best_all
    with open(BEST_PATH, "w") as f:
        json.dump({"params": params, "score": score,
                   "ileri_cm": res[1], "yan_cm": res[2], "donus_deg": res[3], "egim_deg": res[4]}, f, indent=2)
    print("\nEn iyi yürüyüş best_gait_v2.json'a yazıldı:")
    print(json.dumps(params, indent=2))
    print("İzlemek için:  python learn_gait.py --izle")


if __name__ == "__main__":
    if "--izle" in sys.argv:
        data = json.load(open(BEST_PATH))
        print("Yürüyüş:", data["params"])
        simulate(data["params"], gui=True, cycles=400, verbose=True)
    else:
        learn()
