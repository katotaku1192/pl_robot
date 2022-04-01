import cv2
import numpy as np


capture = cv2.VideoCapture(0)
points = np.array([(140, 30), (510, 30), (600, 440), (70, 440)])
pts1 = np.float32([[140, 30], [510, 30], [600, 440], [70, 440]])
pts2 = np.float32([[0, 0], [720,0], [720, 740], [0, 740]])
M = cv2.getPerspectiveTransform(pts1,pts2)



while(True):
    ret, frame = capture.read()
    # resize the window
    windowsize = (1920, 1080)
    #frame = cv2.resize(frame, windowsize)
    cv2.polylines(frame, [points], True, (255, 255, 0))
    cv2.imshow('title',frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord('a'):
        frame_true = cv2.warpPerspective(frame,M,(720,740))
        cv2.imwrite("frame3.png", frame_true)
    elif key == ord('s'):
        frame_false = cv2.warpPerspective(frame,M,(720,740))
        cv2.imwrite("frame4.png", frame_false)

capture.release()
cv2.destroyAllWindows()

