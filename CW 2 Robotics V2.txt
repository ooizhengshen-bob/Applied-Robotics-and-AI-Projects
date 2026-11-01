import random
from controller import Robot

TIME_STEP = 32          
MAX_SPEED = 6.28


OBSTACLE_THRESHOLD = 300
SAFETY_THRESHOLD = 3500


BACKUP_STEPS = 15        # back up for about 0.5 s
TURN_STEPS = 10          # keep turning away for about 0.3 s
WANDER_TURN_EVERY = 150  # random turn about every 5 s
CORNER_MEMORY = 20       # if hit again within this many steps, keep the same turn direction
PRINT_EVERY = 10         # print camera values every 10 steps


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
    
    backup_counter = 0
    avoid_counter = 0
    avoid_dir = 1             # 1 = turn left, -1 = turn right
    steps_since_avoid = 999
    wander_counter = 0
    random_turn_counter = 0
    random_dir = 1
    step_count = 0

    while robot.step(TIME_STEP) != -1:
        step_count += 1

        ps_values = [sensor.getValue() for sensor in ps]
        image = camera.getImage()
        mid_x = width // 2
        mid_y = height // 2
        r = camera.imageGetRed(image, width, mid_x, mid_y)
        g = camera.imageGetGreen(image, width, mid_x, mid_y)
        b = camera.imageGetBlue(image, width, mid_x, mid_y)
        if step_count % PRINT_EVERY == 0:
            print(f"Middle pixel  R={r:3d}  G={g:3d}  B={b:3d}")

        right_side = [ps_values[0], ps_values[1], ps_values[2]]
        left_side = [ps_values[3], ps_values[4], ps_values[5]]
        front = [ps_values[0], ps_values[5]]
        
        if r > 50 and r > g + 50 and r > b+50:
            if "red🍎" not in found_colour:
                print("I see red🍎")
                found_colour.append("red🍎")
                print(f"Yes I found a new colour, it's : {found_colour}")
            
        if b > 50 and b > g + 50 and b > r+50:
            if "blue🧊" not in found_colour:
                print("I see blue🧊")
                found_colour.append("blue🧊")
                print(f"Yes I found a new colour, it's : {found_colour}")
            
        if g > 50 and g > r + 50 and g > b+50:
            if "green🍏" not in found_colour:
                print("I see green🍏")
                found_colour.append("green🍏")
                print(f"Yes I found a new colour, it's : {found_colour}")
            
        # LAYER 0: WANDER
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

        # LAYER 1: OBSTACLE AVOIDANCE
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

        # LAYER 2: SAFETY BACK-UP
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
