import subprocess
import re
import csv
import os
import sys

WORLD_POSE_TOPIC = '/world/terrain_seed_0/pose/info'
ROBOT_NAME = 'burger'
GOAL_NAME = 'goal_marker'
SUCCESS_THRESHOLD = 0.3
RESULTS_FILE = os.path.expanduser('~/Autonomous-Rover-Navigation-in-Simulated-Planetary-Terrain/results/baseline_results.csv')

def get_pose_snapshot():
    result = subprocess.run(['gz', 'topic', '-e', '-t', WORLD_POSE_TOPIC, '-n', '1'], capture_output=True, text=True, timeout=10)
    return result.stdout

def extract_position(text, name):
    pattern = r'name:\s*"' + re.escape(name) + r'".*?position\s*\{([^}]*)\}'
    match = re.search(pattern, text, re.DOTALL)
    if not match:
        return None
    block = match.group(1)
    x_match = re.search(r'x:\s*(-?[\d.eE+-]+)', block)
    y_match = re.search(r'y:\s*(-?[\d.eE+-]+)', block)
    x = float(x_match.group(1)) if x_match else 0.0
    y = float(y_match.group(1)) if y_match else 0.0
    return x, y

def main():
    if len(sys.argv) < 2:
        print('usage: python3 evaluate_trial.py <seed>')
        return
    seed = sys.argv[1]
    text = get_pose_snapshot()
    robot_pos = extract_position(text, ROBOT_NAME)
    goal_pos = extract_position(text, GOAL_NAME)
    if robot_pos is None or goal_pos is None:
        print('could not find robot or goal pose in snapshot')
        return
    dx = goal_pos[0] - robot_pos[0]
    dy = goal_pos[1] - robot_pos[1]
    distance = (dx * dx + dy * dy) ** 0.5
    success = distance < SUCCESS_THRESHOLD
    print(f'seed={seed} robot=({robot_pos[0]:.2f},{robot_pos[1]:.2f}) goal=({goal_pos[0]:.2f},{goal_pos[1]:.2f}) distance={distance:.2f} success={success}')
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    file_exists = os.path.isfile(RESULTS_FILE)
    with open(RESULTS_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(['seed', 'robot_x', 'robot_y', 'goal_x', 'goal_y', 'distance', 'success'])
        writer.writerow([seed, robot_pos[0], robot_pos[1], goal_pos[0], goal_pos[1], distance, success])

if __name__ == '__main__':
    main()
