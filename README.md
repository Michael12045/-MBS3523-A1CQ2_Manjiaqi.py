# -MBS3523-A1CQ2_Manjiaqi.py
# Paste your Python code here:

import cv2
import numpy as np
import serial
import time


ser = serial.Serial('COM13', baudrate=9600, timeout=1)
time.sleep(2)


pan_angle = 90
tilt_angle = 90


cv2.namedWindow('Trackbars')
cv2.createTrackbar('HueLow', 'Trackbars', 18, 255, lambda x: None)
cv2.createTrackbar('HueHigh', 'Trackbars', 32, 255, lambda x: None)
cv2.createTrackbar('SatLow', 'Trackbars', 71, 255, lambda x: None)
cv2.createTrackbar('SatHigh', 'Trackbars', 191, 255, lambda x: None)
cv2.createTrackbar('ValLow', 'Trackbars', 186, 255, lambda x: None)
cv2.createTrackbar('ValHigh', 'Trackbars', 255, 255, lambda x: None)


cam = cv2.VideoCapture(0)


def send_angles(pan, tilt):
    ser.write(f"{pan},{tilt}\n".encode())


while True:
    success, img = cam.read()
    if not success:
        break


    hueLow = cv2.getTrackbarPos('HueLow', 'Trackbars')
    hueHigh = cv2.getTrackbarPos('HueHigh', 'Trackbars')
    satLow = cv2.getTrackbarPos('SatLow', 'Trackbars')
    satHigh = cv2.getTrackbarPos('SatHigh', 'Trackbars')
    valLow = cv2.getTrackbarPos('ValLow', 'Trackbars')
    valHigh = cv2.getTrackbarPos('ValHigh', 'Trackbars')


    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


    lower_bound = np.array([hueLow, satLow, valLow])
    upper_bound = np.array([hueHigh, satHigh, valHigh])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    for cnt in contours:

        area = cv2.contourArea(cnt)
        (x, y, w, h) = cv2.boundingRect(cnt)
        if area > 100:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)

        # largest_contour = max(contours, key=cv2.contourArea)
        # (x, y, w, h) = cv2.boundingRect(largest_contour)


        center_x = x + w // 2
        center_y = y + h // 2


        # cv2.circle(img, (center_x, center_y), 5, (0, 255, 0), 3)


        frame_center_x = img.shape[1] // 2
        frame_center_y = img.shape[0] // 2


        delta_x = center_x - frame_center_x
        delta_y = center_y - frame_center_y


        pan_angle -= delta_x * 0.00025
        tilt_angle -= delta_y * 0.00025


        pan_angle = max(0, min(180, pan_angle))
        tilt_angle = max(0, min(180, tilt_angle))


        send_angles(int(pan_angle), int(tilt_angle))


    cv2.imshow('Mask', mask)
    cv2.imshow('Frame', img)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cam.release()
cv2.destroyAllWindows()
ser.close()


# Paste your Arduino code here:

#include <Servo.h>

Servo panServo;
Servo tiltServo;

int panAngle = 90;
int tiltAngle = 90;

void setup() {
  Serial.begin(9600);
  panServo.attach(9);  //   tiltServo.attach(10); //   panServo.write(panAngle);
  tiltServo.write(tiltAngle);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    panAngle = data.substring(0, commaIndex).toInt();
    tiltAngle = data.substring(commaIndex + 1).toInt();
    
    panServo.write(panAngle);
    tiltServo.write(tiltAngle);
  }
}
