import random
import sys

def generate_bumps(seed, num_bumps=150, world_size=10):
    bumps = ""
    for i in range(num_bumps):
        x = random.uniform(-world_size/2, world_size/2)
        y = random.uniform(-world_size/2, world_size/2)
        radius = random.uniform(0.03, 0.08)
        height = random.uniform(0.02, 0.06)
        bumps += f"""
    <model name='bump_{i}'>
      <static>true</static>
      <pose>{x} {y} {height/2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        </collision>
        <visual name='visual'>
	 <material><ambient>0.6 0.4 0.2 1</ambient><diffuse>0.6 0.4 0.2 1</diffuse></material>
          <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        </visual>
      </link>
    </model>
"""
    return bumps

def generate_world(seed, num_obstacles=10, world_size=10):
    random.seed(seed)
    start_pose = (-world_size/2 + 0.5, 0)
    goal_pose = (world_size/2 - 0.5, 0)
    obstacles = ""
    for i in range(num_obstacles):
        x = random.uniform(-world_size/2, world_size/2)
        y = random.uniform(-world_size/2, world_size/2)
        size = random.uniform(0.2, 0.6)
        obstacles += f"""
    <model name='obstacle_{i}'>
      <static>true</static>
      <pose>{x} {y} {size/2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{size} {size} {size}</size></box></geometry>
        </collision>
        <visual name='visual'>
	<material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
          <geometry><box><size>{size} {size} {size}</size></box></geometry>
        </visual>
      </link>
    </model>
"""
    bumps = generate_bumps(seed, world_size=world_size)
    goal_marker = f"""
    <model name='goal_marker'>
      <static>true</static>
      <pose>{goal_pose[0]} {goal_pose[1]} 0.05 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><cylinder><radius>0.15</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
    start_marker = f"""
    <model name='start_marker'>
      <static>true</static>
      <pose>{start_pose[0]} {start_pose[1]} 0.05 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><cylinder><radius>0.15</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
    world = f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='terrain_seed_{seed}'>
    <light type='directional' name='sun'>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>
    {obstacles}
    {bumps}
    {goal_marker}
    {start_marker}
  </world>
</sdf>
"""
    return world

if __name__ == '__main__':
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    world_xml = generate_world(seed)
    output_path = f"../worlds/terrain_seed_{seed}.sdf"
    with open(output_path, 'w') as f:
        f.write(world_xml)
    print(f"Generated {output_path}")
