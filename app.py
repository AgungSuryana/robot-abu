import cv2
import numpy as np
import paho.mqtt.client as mqtt

# Konfigurasi MQTT
broker = "192.168.133.170"
port = 1884
topic = "robot/status"

def on_connect(client, userdata, flags, rc):
    print("Terhubung ke broker MQTT dengan kode hasil: " + str(rc))

client = mqtt.Client()
client.on_connect = on_connect
client.connect(broker, port, 60)

cap = cv2.VideoCapture(0)

box_size = 40
frame_threshold = 50  # Toleransi horizontal dan vertikal

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_height, frame_width = frame.shape[:2]
    center_frame_x = frame_width // 2
    center_frame_y = frame_height // 2

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 170, 170])
    upper_red1 = np.array([5, 255, 255])
    lower_red2 = np.array([170, 170, 170])
    upper_red2 = np.array([180, 255, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_red:
        largest_contour = max(contours_red, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_object = (x + w // 2, y + h // 2)

            # Gambar crosshair objek
            cv2.line(frame, (center_object[0], 0), (center_object[0], frame_height), (0, 255, 0), 2)
            cv2.line(frame, (0, center_object[1]), (frame_width, center_object[1]), (0, 255, 0), 2)

            # Gambar kotak
            box_top_left = (center_object[0] - box_size//2, center_object[1] - box_size//2)
            box_bottom_right = (center_object[0] + box_size//2, center_object[1] + box_size//2)
            cv2.rectangle(frame, box_top_left, box_bottom_right, (0, 255, 0), 2)

            # Deteksi arah berdasarkan posisi horizontal dan vertikal
            if center_object[0] < center_frame_x - frame_threshold:
                status = "ke Kiri"
                color = (0, 0, 255)
            elif center_object[0] > center_frame_x + frame_threshold:
                status = "ke Kanan"
                color = (0, 0, 255)
            elif center_object[1] < center_frame_y - frame_threshold:
                status = "ke Atas"
                color = (255, 0, 0)
            elif center_object[1] > center_frame_y + frame_threshold:
                status = "ke Bawah"
                color = (255, 0, 0)
            else:
                status = "Tengah"
                color = (0, 255, 0)

            print(status)
            client.publish(topic, status)
            cv2.putText(frame, status, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

    cv2.imshow("Tracking Warna Merah dengan Crosshair Bergerak", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    client.loop()

cap.release()
cv2.destroyAllWindows()
