"""PyBullet demo: hand-tuned trot gait (hip 15 deg, knee 55 deg) with heading correction.

This is the intermediate, hand-tuned version. The gait found by learning is in
best_gait_v2.json and can be watched with:  python learn_gait.py --izle
Usage (run from the simulation/ folder):  python walk_demo.py
"""
import math
import os
import time
import pybullet as p
import pybullet_data

# --- DOSYA ve AYARLAR ---
URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "crawling-panther_v2.urdf")
START_Z = 0.03          # bacak uclari gövde referansının 1.83 cm altında, 3 cm yukseklikten birak

# Gerçek robottaki servo adı -> URDF eklem adı
JOINT_OF = {
    "R1": "base_link_to_hip1", "R2": "base_link_to_hip2",   # sağ kalçalar (ön, arka)
    "R3": "base_link_to_leg1", "R4": "base_link_to_leg2",   # sağ dizler (hip1 ve hip2'nin bacakları)
    "L1": "base_link_to_hip3", "L2": "base_link_to_hip4",   # sol kalçalar (ön, arka)
    "L3": "base_link_to_leg3", "L4": "base_link_to_leg4",   # sol dizler (hip3 ve hip4'ün bacakları)
}

# URDF geometrisinden: dikey eksen (+z) etrafında pozitif dönüş,
# sağ kalçada bacağı İLERİ, sol kalçada GERİ götürür.
HIP_FORWARD_SIGN = {"R1": -1, "R2": -1, "L1": +1, "L2": +1}   # tarama sonucu: ters çıktı

# Diz eklemleri +x ekseni etrafında döner, pozitif dönüş ayağı +y (sola) götürür.
# LIFT_OUTWARD=True: bacak yana (dışarı) kalkar. Gerçek robotta içeri kalkıyorsa False yap.
LIFT_OUTWARD = False   # tarama sonucu: bacaklar içeri kalkınca en iyi yürüyüş
KNEE_OUT_SIGN = {"R3": -1, "R4": -1, "L3": +1, "L4": +1}

HIP_SWING_DEG = 15
KNEE_LIFT_DEG = 55
INTERP_STEPS  = 40            # bir hareket = 40 adım = yaklaşık 0.17 sn benzetim zamanı
STEP_DELAY    = 1.0 / 240.0   # PyBullet'in varsayılan adımı 1/240 sn, gerçek zamanlı izlenir
MAX_FORCE     = 0.25          # N*m, MG90S yaklaşık 0.2
MAX_VELOCITY  = 8.0           # rad/s

SCALE = {"R": 1.0, "L": 1.0}   # sağ/sol adım ölçeği, yön düzeltmesi bunu değiştirir
STEER_GAIN = 0.3               # yön geri besleme kazancı (jiroskop mantığı)

FORWARD  = 1
BACKWARD = -1

# --- PyBullet başlat ---
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

planeId = p.loadURDF("plane.urdf")
p.changeDynamics(
    planeId, -1,
    lateralFriction=2.5,
    spinningFriction=0.5,
    rollingFriction=0.1,
    restitution=0.0
)

startPos = [0, 0, START_Z]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

try:
    robotId = p.loadURDF(URDF_PATH, startPos, startOrientation, useFixedBase=False)
    print(f"'{URDF_PATH}' başarıyla yüklendi.")
except Exception as e:
    print(f"URDF yüklenemedi: {e}")
    p.disconnect()
    raise SystemExit

# Kütlesiz kök parça PyBullet'te robotu sabit yapabilir; kontrol et
base_mass = p.getDynamicsInfo(robotId, -1)[0]
print(f"Kök parça kütlesi: {base_mass} kg")
if base_mass == 0:
    print("UYARI: kök parça (base_footprint) kütlesiz, robot sabit kalabilir. "
          "URDF'den base_footprint ve base_joint'i silip base_link'i kök yap.")

# Eklem adı -> indeks
name_to_index = {}
for i in range(p.getNumJoints(robotId)):
    info = p.getJointInfo(robotId, i)
    name_to_index[info[1].decode()] = i
    print("eklem", i, info[1].decode())

missing = [n for n in JOINT_OF.values() if n not in name_to_index]
if missing:
    print("URDF'de bulunamayan eklemler:", missing)
    p.disconnect()
    raise SystemExit

# Ayak tutuşu
for i in range(p.getNumJoints(robotId)):
    p.changeDynamics(
        robotId, i,
        lateralFriction=0.5,
        spinningFriction=0.2,
        restitution=0.0,
        linearDamping=0.05,
        angularDamping=0.05
    )

current = {k: 0.0 for k in JOINT_OF}   # radyan, 0 = dinlenme duruşu
target  = {k: 0.0 for k in JOINT_OF}
move_dir = FORWARD


def apply_movement():
    """Eklemleri hedef açıya yumuşakça götürür."""
    start = dict(current)
    for s in range(1, INTERP_STEPS + 1):
        for k, joint_name in JOINT_OF.items():
            current[k] = start[k] + (target[k] - start[k]) * s / INTERP_STEPS
            p.setJointMotorControl2(
                bodyIndex=robotId,
                jointIndex=name_to_index[joint_name],
                controlMode=p.POSITION_CONTROL,
                targetPosition=current[k],
                force=MAX_FORCE,
                maxVelocity=MAX_VELOCITY
            )
        p.stepSimulation()
        time.sleep(STEP_DELAY)


def hip(k, d):
    """d=+1 ileri, d=-1 geri."""
    target[k] = HIP_FORWARD_SIGN[k] * math.radians(HIP_SWING_DEG) * SCALE[k[0]] * d * move_dir


def knee(k, lifted):
    out = 1 if LIFT_OUTWARD else -1
    target[k] = KNEE_OUT_SIGN[k] * out * math.radians(KNEE_LIFT_DEG) if lifted else 0.0


def gait_step():
    """Çapraz trot: R1+L2 ve R2+L1 grupları dönüşümlü."""
    # Grup 1: dizler R3 ve L4 kalkar, R1 ve L2 ileri, R2 ve L1 geri
    knee("R3", True); knee("L4", True)
    hip("R1", +1); hip("L2", +1)
    hip("R2", -1); hip("L1", -1)
    apply_movement()

    knee("R3", False); knee("L4", False)
    apply_movement()

    # Grup 2: dizler R4 ve L3 kalkar, R2 ve L1 ileri, R1 ve L2 geri
    knee("R4", True); knee("L3", True)
    hip("R2", +1); hip("L1", +1)
    hip("R1", -1); hip("L2", -1)
    apply_movement()

    knee("R4", False); knee("L3", False)
    apply_movement()


def rest():
    for k in target:
        target[k] = 0.0
    apply_movement()


# --- YÖN DÜZELTMELİ SÜREKLİ YÜRÜYÜŞ ---
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose():
    pos, orn = p.getBasePositionAndOrientation(robotId)
    return pos[0], pos[1], p.getEulerFromQuaternion(orn)[2]


try:
    for _ in range(int(0.7 / STEP_DELAY)):       # yerleşme
        p.stepSimulation()
        time.sleep(STEP_DELAY)
    x0, y0, yaw0 = pose()
    print(f"Yürüyüş başladı: salınım {HIP_SWING_DEG}°, kaldırma {KNEE_LIFT_DEG}°, kazanç {STEER_GAIN}")
    print("Durdurmak için pencereyi kapat / Ctrl+C.")
    n = 0
    while p.isConnected():
        x, y, yaw = pose()
        err = wrap(yaw - yaw0)                    # +: sola sapmış
        steer = STEER_GAIN * (-err)
        SCALE["R"] = min(1.8, max(0.2, 1.0 - steer))
        SCALE["L"] = min(1.8, max(0.2, 1.0 + steer))
        gait_step()
        n += 1
        if n % 5 == 0:
            x, y, yaw = pose()
            print(f"döngü {n:4d} | ileri={100*(x-x0):+7.1f} cm  yan={100*(y-y0):+7.1f} cm  yön={math.degrees(wrap(yaw-yaw0)):+6.1f}°")
except KeyboardInterrupt:
    print("Simülasyon kapatıldı.")
finally:
    if p.isConnected():
        p.disconnect()
