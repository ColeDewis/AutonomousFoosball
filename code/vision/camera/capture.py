import numpy as np
import cv2

from pathlib import Path

file_path = Path(__file__)
img_path = file_path.parent / "images/"

cam = cv2.VideoCapture(2)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
count = 0
img_res = None

# check camera feed and get image resolution
check, tmp = cam.read()
if check == False:
    print("Could not read from camera, exiting ...")
    exit()
img_res = tmp.shape[:2]
print(img_res)

# keep going until user quits
while True:
    _, img = cam.read()
    img_copy = img.copy()
    cv2.putText(img_copy, f"Captured image: {count}. Press 'c' to capture image. Press 'q' to quit.", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0))
    cv2.imshow("image", img_copy)

    key = cv2.waitKey(10)
    if key == ord('c'):
        fname = str(img_path) + f"/{count}_{img_res[1]}x{img_res[0]}" + ".png"
        # fname = str(img_path) + f"/test" + ".png"
        cv2.imwrite(fname, img)
        count += 1
    if key == ord('q'):
        break;

cv2.destroyAllWindows()
