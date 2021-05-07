import cv2
import pigpio
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np

frameHeight = 640
frameWidth = 480


def empty():
    pass


def servo_test_gui():
    cv2.namedWindow("ServoControl")
    cv2.resizeWindow("ServoControl", 640, 240)
    cv2.createTrackbar("PanControl", "ServoControl", 0, 180, empty)
    cv2.createTrackbar("TiltControl", "ServoControl", 0, 180, empty)


def get_contours(mask, img):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h = 0, 0, 0, 0
    if len(contours) != 0:
        maxContour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img, maxContour, -1, (255, 255, 255), 3)
        return x, y, w, h, img


def track_color(img, colorDetected):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array(colorDetected[0:3])
    upper = np.array(colorDetected[3:6])
    mask = cv2.inRange(imgHSV, lower, upper)
    if cv2.countNonZero(mask) == 0:
        return 0, 0, 0, 0, img
    else:
        x, y, w, h, imgContour = get_contours(mask, img)
        cv2.rectangle(imgContour, (x, y), (x+w, y+h), (200, 200, 200), 2)

    return x, y, w, h, imgContour


def track_with_servos(x, y, w, h):
    if w == 0 or h == 0:
        panIncrement = 0
        tiltIncrement = 0
    else:
        objX = x + w//2
        objY = y + h//2
        centerX = frameWidth//2
        centerY = frameHeight//2
        distanceFromCenterX = objX - centerX
        distanceFromCenterY = objY - centerY
        if np.abs(distanceFromCenterX) >= 10:
            panIncrement = float(distanceFromCenterX) / 5.
        if np.abs(distanceFromCenterY) >= 10:
            tiltIncrement = float(distanceFromCenterY) / 7.
    return -panIncrement, tiltIncrement


def main():
    # GUI SHI
    # servo_test_gui()

    # CAMERA SHI
    camera = PiCamera()
    camera.resolution = (frameHeight, frameWidth)
    camera.framerate = 32
    camera.rotation = 180
    rawCapture = PiRGBArray(camera, size=(frameHeight, frameWidth))
    time.sleep(0.1)

    # CV SHI
    colorDetected = [89, 64, 110, 100, 255, 255]

    # RPI SHI
    pi = pigpio.pi()

    # SERVO SHI
    pan = 17
    tilt = 27
    pi.set_mode(pan, pigpio.OUTPUT)
    pi.set_mode(tilt, pigpio.OUTPUT)
    pi.set_PWM_frequency(pan, 50)
    pi.set_PWM_frequency(tilt, 50)
    panAngle = 90
    tiltAngle = 60

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        # READ FROM CAP
        img = frame.array

        # CV IN ACTION
        x, y, w, h, imgResult = track_color(img, colorDetected)
        panIncrement, tiltIncrement = track_with_servos(x, y, w, h)

        # SHOW IMG
        cv2.imshow("Video", imgResult)

        # CALCULATING ANGLE AND INCREMENTS
        if panAngle >= 170 and panIncrement > 0:
            panIncrement = 0
        elif panAngle <= 10 and panIncrement < 0:
            panIncrement = 0
        if tiltAngle >= 130 and tiltIncrement > 0:
            tiltIncrement = 0
        elif tiltAngle <= 10 and tiltIncrement < 0:
            tiltIncrement = 0

        print(panAngle, "\t", panIncrement, tiltAngle, "\t", tiltIncrement)
        panAngle += panIncrement
        tiltAngle += tiltIncrement

        panPulse = ((2000. * panAngle) / 180.) + 500.
        tiltPulse = ((2000. * tiltAngle) / 180.) + 500.

        pi.set_servo_pulsewidth(pan, panPulse)
        pi.set_servo_pulsewidth(tilt, tiltPulse)

        # CLEAR CAMERA
        rawCapture.truncate(0)
        if cv2.waitKey(1) & 0xFF ==ord('q'):
            pi.set_PWM_dutycycle(pan, 0)
            pi.set_PWM_dutycycle(tilt, 0)
            pi.set_PWM_frequency(pan, 0)
            pi.set_PWM_frequency(tilt, 0)
            camera.close()
            break


if __name__ == '__main__':
    main()
