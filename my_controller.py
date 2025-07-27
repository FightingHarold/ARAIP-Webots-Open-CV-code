""" Robot obstacle avoidance, color detection, and horse image saving with OpenCV """
from controller import Robot
import cv2
import numpy as np

# Initialize robot and peripherals
robot = Robot()
print("Robot instance created.")

TIME_STEP = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(4.0)
right_motor.setVelocity(4.0)

# Distance sensors
distance_sensors = []
sensor_names = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)

# Camera
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

Obstacle_Threshold = 80
Turn_steps = 25
Turn_counter = 0
is_turning = False
seen_colors = set()
image_saved = False  # Prevents saving multiple images

def detect_color_opencv(image, width, height, x, y):
    idx = (y * width + x) * 4
    pixel_bytes = image[idx:idx+4]  # [B, G, R, A]
    b, g, r, a = [int(pixel_bytes[i]) for i in range(4)]
    pixel_bgr = np.uint8([[[b, g, r]]])
    hsv = cv2.cvtColor(pixel_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[0][0]
    if ((h >= 0 and h <= 10) or (h >= 160 and h <= 180)) and s > 100 and v > 50:
        return 'red'
    elif (h >= 40 and h <= 85) and s > 100 and v > 50:
        return 'green'
    elif (h >= 90 and h <= 130) and s > 100 and v > 50:
        return 'blue'
    else:
        return None

while robot.step(TIME_STEP) != -1:
    distances = [sensor.getValue() for sensor in distance_sensors]
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    x, y = width // 2, height // 2

    # Pixel color reading
    r = camera.imageGetRed(image, width, x, y)
    g = camera.imageGetGreen(image, width, x, y)
    b = camera.imageGetBlue(image, width, x, y)

    # Basic RGB color detection
    detected_color = None
    if r > 100 and g < 80 and b < 80:
        detected_color = 'red'
    elif g > 100 and r < 80 and b < 80:
        detected_color = 'green'
    elif b > 100 and r < 80 and g < 80:
        detected_color = 'blue'
    if detected_color and detected_color not in seen_colors:
        print(f"I see {detected_color}")
        seen_colors.add(detected_color)
        print("Summary: I have previously seen " + ", ".join(seen_colors))
        print(f"R={r}, G={g}, B={b}")

    # OpenCV HSV color detection for center pixel (debug every loop)
    opencv_color = detect_color_opencv(image, width, height, x, y)
    print(f"[DEBUG - OpenCV] Pixel (x={x}, y={y}), BGR=({b},{g},{r}) -> Detected color: {opencv_color}")
    if opencv_color and opencv_color not in seen_colors:
        print(f"OpenCV detects: {opencv_color}")
        seen_colors.add(opencv_color)
        print("Summary (OpenCV):", ", ".join(seen_colors))
        print(f"HSV-based detection at R={r}, G={g}, B={b}")

    # Horse (brown region) detection and image saving
    img_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    img_bgr = img_array[:, :, :3]  # Discard alpha

    avg_b, avg_g, avg_r = np.mean(img_bgr, axis=(0, 1))
    # Thresholds can be tuned for your specific horse model and world lighting
    if (avg_r > 80 and avg_r < 180 and
        avg_g > 40 and avg_g < 140 and
        avg_b > 20 and avg_b < 110 and
        (avg_r > avg_g) and (avg_r > avg_b)
        and not image_saved):
        print("Seen a deer! Saving the image...")
        resized_img = cv2.resize(img_bgr, (32, 32))
        cv2.imwrite("webots_deer_detected.png", resized_img)
        print("dog image saved as 'webots_deer_detected.png'")
        image_saved = True

    # Obstacle avoidance
    front_obstacle = any(sensor_value > Obstacle_Threshold for sensor_value in distances)
    if is_turning:
        Turn_counter += 1
        if Turn_counter >= Turn_steps:
            is_turning = False
            Turn_counter = 0
            left_motor.setVelocity(5.0)
            right_motor.setVelocity(5.0)
        else:
            left_motor.setVelocity(4.0)
            right_motor.setVelocity(-4.0)
    else:
        if front_obstacle:
            print("Obstacle seen! Turning")
            is_turning = True
            Turn_counter = 0
            left_motor.setVelocity(4.0)
            right_motor.setVelocity(-4.0)
        else:
            left_motor.setVelocity(5.0)
            right_motor.setVelocity(5.0)

# Uncomment if using cv2.imshow for live video window
# cv2.destroyAllWindows()

