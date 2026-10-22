# Autonomous Rover Navigation in Simulated Planetary Terrain

**Status: In progress.** The rule-based controller and simulation setup are built. The RL policy has not been started yet.

The goal of this project is to compare two ways of getting a rover to drive to a goal across rough, unfamiliar terrain in simulation: a hand-designed rule-based controller and a reinforcement learning (RL) policy. Each test world is generated from a random seed, so the rover can be tested on terrain it has never seen before.

## Current Progress

Done:
- Seeded terrain generator for Gazebo worlds
- Rule-based controller (potential fields with a wall-following fallback)
- Launch file for spawning the rover in a generated world
- Evaluation script that records whether a run reached the goal
- First baseline trial

Not started yet:
- PPO (RL) policy
- Full baseline across all test seeds
- Comparison between the two methods

## Setup

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- TurtleBot3 Burger (simulated), with a 2D LiDAR and wheel odometry

## How It Works

### Terrain generation

`scripts/generate_terrain.py` builds a 10 m by 10 m Gazebo world from a seed. Each world has:

- 10 box obstacles of random size (0.2 to 0.6 m) placed at random
- 150 small bumps scattered across the ground to make the surface uneven
- A blue start marker at (-4.5, 0) and a green goal marker at (4.5, 0)

The same seed always produces the same world, so every trial can be repeated exactly.

### Training and test seeds

`scripts/config.py` sets aside seeds 0 to 900 for training the RL policy later, and holds out seeds 901 to 905 for final testing. The held-out seeds will never be used in training, so the final results will show how each method handles terrain it has never seen.

### Rule-based controller

`scripts/rule_based_controller.py` is a ROS 2 node that uses a potential field approach:

- The goal pulls the rover toward it.
- Any obstacle the LiDAR sees within 0.6 m pushes the rover away, with an extra sideways push so it slides around obstacles instead of stopping in front of them.
- If the rover moves less than 5 cm over 30 control steps, it decides it is stuck and switches to a wall-following mode. It follows the obstacle edge in whichever direction points closer to the goal, then goes back to heading for the goal once it is clear and has made progress, or after 300 steps.

The rover stops when it is within 0.2 m of the goal. Speed is capped at 0.2 m/s.

### Evaluation

`scripts/evaluate_trial.py` reads the rover and goal positions from Gazebo at the end of a run, computes the distance between them, and counts the trial as a success if the rover finished within 0.3 m of the goal. Each result is added as a row in `results/baseline_results.csv`.

## Running It

Generate a world (run from the `scripts` folder):

```bash
cd ~/Autonomous-Rover-Navigation-in-Simulated-Planetary-Terrain/scripts
python3 generate_terrain.py 0
```

Launch the simulation (run from the main project folder):

```bash
cd ~/Autonomous-Rover-Navigation-in-Simulated-Planetary-Terrain
export TURTLEBOT3_MODEL=burger
ros2 launch launch/spawn_terrain_seed.launch.py world:=$(pwd)/worlds/terrain_seed_0.sdf
```

In a second terminal, start the controller:

```bash
cd ~/Autonomous-Rover-Navigation-in-Simulated-Planetary-Terrain/scripts
python3 rule_based_controller.py
```

When the run ends, record the result:

```bash
python3 evaluate_trial.py 0
```

## Results So Far

Only one baseline trial has been run so far.

| Seed | Final distance to goal | Success |
|------|------------------------|---------|
| 0    | 3.57 m                 | No      |

## Next Steps

- Run the rule-based baseline across all held-out seeds
- Train a PPO (a common RL algorithm) policy on the training seeds and compare it with the rule-based controller on the held-out seeds
- Move from generated terrain to real Mars elevation maps from HiRISE, training on Jezero Crater and testing on Gale Crater
- Compare navigation using 2D LiDAR versus a depth camera

## Author

Angela Iweka, Arizona State University
Advised by Professor Kunal Garg
Project page: https://angelaiweka.github.io/rover-research
