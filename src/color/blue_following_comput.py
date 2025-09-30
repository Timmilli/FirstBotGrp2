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
    if y < height / 3:
        return True
    return False


def coord_is_in_center(coord):
    x, y = coord
    if height / 3 <= y and y <= height * 2 / 3:
        return True
    return False


def coord_is_in_right(coord):
    x, y = coord
    if height * 2 / 3 < y:
        return True
    return False


# To be encoded in BGR
boundaries = [
    ([190, 150, 0], [255, 200, 50]),  # A blue tape
]


lower, upper = boundaries[0]
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")

while True:
    result, frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    frame = frame[0:height, int(width/2)-5:int(width/2)+5]

    # Mask and output for color to be followed
    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask=mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    nb_left = 0
    nb_center = 0
    nb_right = 0
    # If there are coordinates detected
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
    # In case if nothing is detected
    else:
        coords = [0]
    # Percentages of mask pixels in a zone of the image
    print(nb_left/len(coords), nb_center/len(coords), nb_right/len(coords))
    if MOTOR_USED:
        dxl_io.set_moving_speed(
            {1: STANDARD_SPEED - nb_left/len(coords)*STANDARD_SPEED})  # Degrees / s
        dxl_io.set_moving_speed(
            {2: -(STANDARD_SPEED - nb_right/len(coords)*STANDARD_SPEED)})  # Degrees / s

    # Visual line, not necessary for computing
    cv2.line(output, (0, int(height/3)),
             (width, int(height/3)), (255, 0, 0), 2)
    cv2.line(output, (0, int(height*2/3)),
             (width, int(height*2/3)), (255, 0, 0), 2)

    # Showing images
    cv2.imshow("images", np.hstack([frame, output]))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()
if MOTOR_USED:
    dxl_io.set_moving_speed(
        {1: 0})  # Degrees / s
    dxl_io.set_moving_speed(
        {2: 0})  # Degrees / s
