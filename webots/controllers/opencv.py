
"""my_arap controller + RED detection + OpenCV image saving (e-puck)"""
from controller import Robot
import cv2 as cv
import numpy as np
import os

TIME_STEP = 32
MAX_SPEED = 6.28
DIST_LIMIT = 70.0

# RED detection thresholds
R_MIN = 100
G_MAX = 80
B_MAX = 80

robot = Robot()

# Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Proximity sensors
rightSensor1 = robot.getDevice('ps0')
rightSensor2 = robot.getDevice('ps1')
leftSensor1 = robot.getDevice('ps7')
leftSensor2 = robot.getDevice('ps6')

rightSensor1.enable(TIME_STEP)
rightSensor2.enable(TIME_STEP)
leftSensor1.enable(TIME_STEP)
leftSensor2.enable(TIME_STEP)

# Camera
cam = robot.getDevice('camera')
cam.enable(TIME_STEP)

W = cam.getWidth()
H = cam.getHeight()

# Create folder to save images
SAVE_FOLDER = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(SAVE_FOLDER, exist_ok=True)

image_count = 0
red_detected_before = False


# ----------------------------------
# Proximity sensor functions
# ----------------------------------

def get_left_sensor():
    value1 = leftSensor1.getValue()
    value2 = leftSensor2.getValue()
    average = (value1 + value2) / 2.0
    return average > DIST_LIMIT


def get_right_sensor():
    value1 = rightSensor1.getValue()
    value2 = rightSensor2.getValue()
    average = (value1 + value2) / 2.0
    return average > DIST_LIMIT


# ----------------------------------
# RED object detection
# ----------------------------------

def red_seen_center():

    img = cam.getImage()

    if img is None:
        return False

    x = W // 2
    y = H // 2

    r = cam.imageGetRed(img, W, x, y)
    g = cam.imageGetGreen(img, W, x, y)
    b = cam.imageGetBlue(img, W, x, y)

    return (r > R_MIN) and (g < G_MAX) and (b < B_MAX)


# ----------------------------------
# Capture and save camera image
# ----------------------------------

def save_camera_image():

    global image_count

    img = cam.getImage()

    if img is None:
        print("Camera image unavailable")
        return

    # Convert Webots image to NumPy array
    image_array = np.frombuffer(img, dtype=np.uint8)
    image_array = image_array.reshape((H, W, 4))

    # Convert BGRA to BGR
    image_bgr = cv.cvtColor(image_array, cv.COLOR_BGRA2BGR)

    # Image filename
    image_count += 1

    filename = f"red_object_{image_count:03d}.jpg"

    filepath = os.path.join(SAVE_FOLDER, filename)

    # Save image using OpenCV
    success = cv.imwrite(filepath, image_bgr)

    if success:
        print("Image saved:", filepath)
    else:
        print("Failed to save image")


# ----------------------------------
# Robot movement functions
# ----------------------------------

def stop():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)


def move_forward():
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)


def move_backward():
    leftMotor.setVelocity(-0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)


def turn_left():
    leftMotor.setVelocity(0.1 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)


def turn_right():
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.1 * MAX_SPEED)


# ----------------------------------
# Main controller
# ----------------------------------

def main():

    global red_detected_before

    TIME_DELAY = 0.5
    time_start = robot.getTime()

    move_forward()

    while robot.step(TIME_STEP) != -1:

        # Priority 1: RED detection
        if red_seen_center():

            stop()

            if not red_detected_before:

                print("RED detected! Stopping.")

                # Capture and save image
                save_camera_image()

                red_detected_before = True

            continue

        # Reset detection flag
        red_detected_before = False

        # Priority 2: Obstacle avoidance
        time_elapsed = robot.getTime()
        time_diff = time_elapsed - time_start

        if time_diff >= TIME_DELAY:

            blocked_left = get_left_sensor()
            blocked_right = get_right_sensor()

            if blocked_left and blocked_right:
                move_backward()

            elif blocked_left:
                turn_right()

            elif blocked_right:
                turn_left()

            else:
                move_forward()

            time_start = time_elapsed


if __name__ == '__main__':
    main()
