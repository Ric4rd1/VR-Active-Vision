import cv2

url = "http://192.168.1.78:5000/"

# Open the MJPEG stream
cap = cv2.VideoCapture(url)

if not cap.isOpened():
    print("Failed to open stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Flask Video Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
