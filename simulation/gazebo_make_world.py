"""Gazebo (gz-sim) dünya dosyasını üretir: zemin + crawling-panther_v2.urdf + 8 eklem kontrolcüsü.

Kullanım:  python3 gazebo_make_world.py
Çıktı:     simulation/robot_world_v2.sdf
Başlatma:  gz sim -r simulation/robot_world_v2.sdf
"""
import os
import sys

HOME = os.path.expanduser("~")
URDF = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "urdf", "crawling-panther_v2.urdf"))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_world_v2.sdf")

# kullanım: python3 gazebo_make_world.py [p_gain] [d_gain]
P_GAIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
D_GAIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02

# eklem adı -> komut konusu
TOPICS = {
    "base_link_to_hip1": "/cmd/hip1", "base_link_to_hip2": "/cmd/hip2",
    "base_link_to_hip3": "/cmd/hip3", "base_link_to_hip4": "/cmd/hip4",
    "base_link_to_leg1": "/cmd/leg1", "base_link_to_leg2": "/cmd/leg2",
    "base_link_to_leg3": "/cmd/leg3", "base_link_to_leg4": "/cmd/leg4",
}

plugins = ""
for joint, topic in TOPICS.items():
    plugins += f"""
      <plugin filename="gz-sim-joint-position-controller-system"
              name="gz::sim::systems::JointPositionController">
        <joint_name>{joint}</joint_name>
        <topic>{topic}</topic>
        <p_gain>{P_GAIN}</p_gain>
        <d_gain>{D_GAIN}</d_gain>
        <cmd_max>0.25</cmd_max>
        <cmd_min>-0.25</cmd_min>
      </plugin>"""

sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="spider_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    <include>
      <uri>file://{URDF}</uri>
      <name>spider</name>
      <pose>0 0 0.03 0 0 0</pose>{plugins}
    </include>
  </world>
</sdf>
"""
with open(OUT, "w", encoding="utf-8") as f:
    f.write(sdf)
print("Yazıldı:", OUT, f"(p_gain={P_GAIN}, d_gain={D_GAIN})")
print("Başlat: gz sim -r", OUT)
