import numpy as np
import cv2

video_capture = cv2.VideoCapture(0)

boundaries = [
    ([17, 15, 100], [50, 56, 200]),
    ([0, 200, 0], [255, 255, 255]),
    ([50, 150, 0], [150, 255, 50]),
    ([86, 31, 4], [220, 88, 50]),
    ([25, 146, 190], [62, 174, 250]),
    ([103, 86, 65], [145, 133, 128])
]

lower, upper = boundaries[2]
lower = np.array(lower, dtype="uint8")
upper = np.array(upper, dtype="uint8")

while True:
    result, frame = video_capture.read()  # read frames from the video
    if result is False:
        break  # terminate the loop if the frame is not read successfully

    mask = cv2.inRange(frame, lower, upper)
    output = cv2.bitwise_and(frame, frame, mask=mask)
    # show the images
    coords = cv2.findNonZero(mask)
    if coords is not None:
        for coord in coords:
            output = cv2.circle(output, coord[0], radius=0,
                                color=(0, 0, 255), thickness=-1)

    cv2.imshow("images", np.hstack([frame, output]))

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()
