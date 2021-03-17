import cv2

source = "rtsp://192.168.43.43:8554/stream"
cap = cv2.VideoCapture(source)

if cap.isOpened():
        cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
        while True:
                ret_val, img = cap.read();
                cv2.imshow('demo',img)
                cv2.waitKey(10)
else:
        print ("camera open failed")

cv2.destroyAllWindows()