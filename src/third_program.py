import numpy as np
import cv2

video_capture = cv2.VideoCapture(0)

boundaries = [
    ([0, 200, 0], [255, 255, 255]),  # A red
    ([50, 150, 0], [150, 255, 50]),  # A green
]

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


lower, upper = boundaries[1]  # Select the color to be detected
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")

while True:
    result, frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask=mask)
    # Find all pixels detected in the mask
    coords = cv2.findNonZero(mask)
    # In case if nothing is detected
    if coords is not None:
        nb_left = 0
        nb_center = 0
        nb_right = 0
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
        # Percentages of mask pixels in a zone of the image
        print(nb_left/len(coords), nb_center/len(coords), nb_right/len(coords))

    # Visual line, not necessary for computing
    cv2.line(output, (int(width/3), 0),
             (int(width/3), height), (255, 0, 0), 2)
    cv2.line(output, (int(width*2/3), 0),
             (int(width*2/3), height), (255, 0, 0), 2)

    # Showing images
    cv2.imshow("images", np.hstack([frame, output]))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()
