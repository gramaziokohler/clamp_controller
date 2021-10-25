import cv2, time
print ("starting grab")
cap = cv2.VideoCapture('http://192.168.1.100')
tick = 0

def put_text(frame, text, org):
    frame = cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 2, cv2.LINE_AA)
    return cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)

while True:
    start = time.time()
    ret, frame = cap.read()
    print("Frame %i arrived after %.2f"% (tick, (time.time() - start)))
    tick +=1

    fps = 1 / (time.time() - start)
    put_text(frame, '%.1fFPS (Frame%i)' % (fps, tick), (20,40))
    cv2.imshow('Video', frame)

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break

cap.release()
print ("after release")