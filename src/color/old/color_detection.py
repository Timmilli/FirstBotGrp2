import numpy as np
import cv2
import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
if not ports:
    MOTOR_USED = False
    print("Motor not used.")
else:
    MOTOR_USED = True
    print("Motor used.")

if MOTOR_USED:
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1])
    STANDARD_SPEED = 360

video_capture = cv2.VideoCapture(0)

width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(video_capture.get(cv2.CAP_PROP_FPS))
print(f"width:{width}; height:{height}; fps:{fps}")


def coord_is_in_left(coord):
    x, y = coord
    if x < width / 3:
        return True
    return False


def coord_is_in_center(coord):
    x, y = coord
    if width / 3 <= x and x <= width * 2 / 3:
        return True
    return False


def coord_is_in_right(coord):
    x, y = coord
    if width * 2 / 3 < x:
        return True
    return False


def coord_is_in_brown_rect(coord):
    x, y = coord
    if width / 3 < x and x < width * 2 / 3:
        if height / 3 < y and y < height * 2 / 3:
            return True
    return False


color_string = ["A yellow tape", "A blue tape", "A red tape"]
current_color = 0

# To be encoded in BGR
boundaries = [
    ([100, 170, 150], [150, 220, 200]),  # A yellow tape (to be reworked on)
    ([190, 150, 0], [255, 200, 50]),  # A blue tape
    ([140, 70, 130], [100, 100, 255]),  # A red tape (to be reworked on)
]
brown_boundaries = [
    ([110, 90, 70], [130, 110, 90]),  # A brown tape (to be reworked on)
]


def next_color():
    global current_color
    if current_color < len(boundaries)-1:
        current_color += 1
    else:
        current_color = 0
    print(color_string[current_color])
    return current_color


lower, upper = boundaries[next_color()]
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")
brown_lower, brown_upper = brown_boundaries[0]
brown_lower = np.array(brown_lower, dtype="uint8")
brown_upper = np.array(brown_upper, dtype="uint8")

switched = False

while True:
    result, frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    # Mask and output for color to be followed
    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask=mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    # In case if nothing is detected
    nb_left = 0
    nb_center = 0
    nb_right = 0
    if coords is not None:
        for coord in coords:
            # If the coordinate is in the first (left) third
            if coord_is_in_left(coord[0]):
                # Show the coordinate as red
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 0, 255), thickness=-1)
                # Add the pixel to the left counter
                nb_left += 1
            elif coord_is_in_center(coord[0]):
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(0, 255, 0), thickness=-1)
                nb_center += 1
            elif coord_is_in_right(coord[0]):
                output = cv2.circle(output, coord[0], radius=0,
                                    color=(255, 0, 0), thickness=-1)
                nb_right += 1
    else:
        coords = [0]
    # Percentages of mask pixels in a zone of the image
    print(nb_left/len(coords), nb_center/len(coords), nb_right/len(coords))
    if MOTOR_USED:
        dxl_io.set_moving_speed(
            {1: STANDARD_SPEED - nb_left/len(coords)*STANDARD_SPEED})  # Degrees / s
        dxl_io.set_moving_speed(
            {2: STANDARD_SPEED - nb_right/len(coords)*STANDARD_SPEED})  # Degrees / s

    # Visual line, not necessary for computing
    cv2.line(output, (int(width/3), 0),
             (int(width/3), height), (255, 0, 0), 2)
    cv2.line(output, (int(width*2/3), 0),
             (int(width*2/3), height), (255, 0, 0), 2)

    # Brown tape related mask and output
    brown_mask = cv2.inRange(frame, brown_lower, brown_upper)
    brown_output = cv2.bitwise_and(frame, frame, mask=brown_mask)
    # Find all pixels detected in the mask
    brown_coords = cv2.findNonZero(brown_mask)
    cv2.rectangle(brown_output, (int(width/3), int(height/3)),
                  (int(width*2/3), int(height*2/3)), (255, 0, 0), 2)
    if brown_coords is not None:
        for brown_coord in brown_coords:
            if coord_is_in_brown_rect(brown_coord[0]):
                brown_output = cv2.circle(brown_output, brown_coord[0], radius=0,
                                          color=(255, 255, 255), thickness=-1)
                if not switched:
                    lower, upper = boundaries[next_color()]
                    lower = np.array(lower, dtype="uint8")
                    upper = np.array(upper, dtype="uint8")
                    switched = True

    # Showing images
    cv2.imshow("images", np.hstack([frame, output, brown_output]))

    if cv2.waitKey(1) & 0xFE == ord("d"):
        # Select the color to be detected
        switched = False

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()
if MOTOR_USED:
    dxl_io.set_moving_speed(
        {1: 0})  # Degrees / s
    dxl_io.set_moving_speed(
        {2: 0})  # Degrees / s
