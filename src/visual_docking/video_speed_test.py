import cv2, time
print ("starting grab")
cap = cv2.VideoCapture('http://192.168.1.102')
tick = 0
while True:
    start = time.time()
    ret, frame = cap.read()
    print("Frame %i arrived after %.2f"% (tick, (time.time() - start)))
    tick +=1
    cv2.imshow('Video', frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

cap.release()
print ("after release")