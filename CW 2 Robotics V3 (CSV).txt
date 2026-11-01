import os
import csv
import random
from datetime import datetime
from controller import Robot

TIME_STEP = 32
MAX_SPEED = 6.28

OBSTACLE_THRESHOLD = 300
SAFETY_THRESHOLD = 3500

BACKUP_STEPS = 15
TURN_STEPS = 10
WANDER_TURN_EVERY = 150
CORNER_MEMORY = 20

METHOD = "RGB"
TIME_LIMIT = 300
RESULTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results.csv")
CSV_HEADER = ["run_id", "method", "colours_found",
              "time_red", "time_blue", "time_green", "total_time_s"]


def save_result(run_id, found_times, total_time):
    file_is_new = not os.path.exists(RESULTS_FILE)
    with open(RESULTS_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if file_is_new:
            writer.writerow(CSV_HEADER)
        writer.writerow([
            run_id,
            METHOD,
            len(found_times),
            found_times.get("red", ""),
            found_times.get("blue", ""),
            found_times.get("green", ""),
            round(total_time, 1),
        ])
    print(f"Result saved to {RESULTS_FILE}")


def run_robot(robot):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    ps_indices = [0, 1, 2, 5, 6, 7]
    ps = []
    for i in ps_indices:
        sensor = robot.getDevice(f'ps{i}')
        sensor.enable(TIME_STEP)
        ps.append(sensor)

    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)
    width = camera.getWidth()
    height = camera.getHeight()

    found_colour = []
    found_times = {}
    run_id = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    result_saved = False

    backup_counter = 0
    avoid_counter = 0
    avoid_dir = 1
    steps_since_avoid = 999
    wander_counter = 0
    random_turn_counter = 0
    random_dir = 1

    while robot.step(TIME_STEP) != -1:
        now = robot.getTime()

        ps_values = [sensor.getValue() for sensor in ps]

        image = camera.getImage()
        mid_x = width // 2
        mid_y = height // 2
        r = camera.imageGetRed(image, width, mid_x, mid_y)
        g = camera.imageGetGreen(image, width, mid_x, mid_y)
        b = camera.imageGetBlue(image, width, mid_x, mid_y)

        right_side = [ps_values[0], ps_values[1], ps_values[2]]
        left_side = [ps_values[3], ps_values[4], ps_values[5]]
        front = [ps_values[0], ps_values[5]]

        if r > 50 and r > g + 50 and r > b + 50:
            if "red" not in found_colour:
                print("I see red")
                found_colour.append("red")
                found_times["red"] = round(now, 1)
                print(f"Found so far: {found_colour}")

        if b > 50 and b > g + 50 and b > r + 50:
            if "blue" not in found_colour:
                print("I see blue")
                found_colour.append("blue")
                found_times["blue"] = round(now, 1)
                print(f"Found so far: {found_colour}")

        if g > 50 and g > r + 50 and g > b + 50:
            if "green" not in found_colour:
                print("I see green")
                found_colour.append("green")
                found_times["green"] = round(now, 1)
                print(f"Found so far: {found_colour}")

        if not result_saved:
            if len(found_colour) == 3:
                print(f"All three colours found in {now:.1f} s")
                save_result(run_id, found_times, now)
                result_saved = True
            elif now >= TIME_LIMIT:
                print(f"Time limit reached. Found {len(found_colour)} of 3 colours.")
                save_result(run_id, found_times, now)
                result_saved = True

        left_speed = 0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
        wander_counter += 1
        if random_turn_counter > 0:
            left_speed = -0.5 * MAX_SPEED * random_dir
            right_speed = 0.5 * MAX_SPEED * random_dir
            random_turn_counter -= 1
        elif wander_counter >= WANDER_TURN_EVERY:
            wander_counter = 0
            random_dir = random.choice([1, -1])
            random_turn_counter = random.randint(5, 20)

        if avoid_counter == 0 and any(value > OBSTACLE_THRESHOLD for value in ps_values):
            if steps_since_avoid > CORNER_MEMORY:
                if max(right_side) > max(left_side):
                    avoid_dir = 1
                else:
                    avoid_dir = -1
            avoid_counter = TURN_STEPS

        if avoid_counter > 0:
            left_speed = -0.5 * MAX_SPEED * avoid_dir
            right_speed = 0.5 * MAX_SPEED * avoid_dir
            avoid_counter -= 1
            steps_since_avoid = 0
        else:
            steps_since_avoid += 1

        if backup_counter == 0 and any(value > SAFETY_THRESHOLD for value in front):
            backup_counter = BACKUP_STEPS

        if backup_counter > 0:
            left_speed = -0.5 * MAX_SPEED
            right_speed = -0.5 * MAX_SPEED
            backup_counter -= 1

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == '__main__':
    robot = Robot()
    run_robot(robot)
