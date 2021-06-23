# Gerekli kütüphaneleri Ekliyoruz
import cv2
import sys
import smbus
import math
import RPi.GPIO as GPIO
import time
import pygame

# Üzerinde bulunan kamera modülünden görüntü alabilmesi için gerekli tanımlamaları yapıyoruz
# raspi-config üzerinden camera kısmını enable etmeniz gerekiyor.
from picamera.array import PiRGBArray
from picamera import PiCamera

# Gerekli pin ve çıkışları tanımlıyoruz
GPIO.setmode(GPIO.BCM)
buttonPin = 21
GPIO.setup(buttonPin,GPIO.IN, pull_up_down=GPIO.PUD_UP)

in1 = 24
in2 = 23
in3 = 27
in4 = 22
ena = 25
enb = 17
temp1=1

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(ena,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

p1=GPIO.PWM(ena,1000)
p2=GPIO.PWM(enb,1000)
p1.start(25)
p2.start(25)

# Görüntünün çözünürlüğünü ve fps değerlerini tanımlıyoruz
faceDetect = False
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# opencv üzerinden tanımlayacağımız nesne için oluşturulan xml dosyasını tanımlıyoruz.
cascPath = 'HS.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

# MPU6050'yi bağladığımız i2c portlarını tanımlıyoruz.
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1)
address = 0x68 #MPU6050 I2C adresi
#MPU6050 ilk calistiginda uyku modunda oldugundan, calistirmak icin asagidaki komutu veriyoruz:
bus.write_byte_data(address, power_mgmt_1, 0)

def read_byte(adr):
 return bus.read_byte_data(address, adr)

def read_word(adr):
 high = bus.read_byte_data(address, adr)
 low = bus.read_byte_data(address, adr+1)
 val = (high << 8) + low
 return val

def read_word_2c(adr):
 val = read_word(adr)
 if (val >= 0x8000):
     return -((65535 - val) + 1)
 else:
     return val

def dist(a,b):
 return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
 radians = math.atan2(x, dist(y,z))
 return -math.degrees(radians)

def get_x_rotation(x,y,z):
 radians = math.atan2(y, dist(x,z))
 return math.degrees(radians)



# for döngüsü ile gelen görüntünün her karesini tanıyoruz.
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(5, 5),
        flags=cv2.CASCADE_SCALE_IMAGE
    )

    # Vücut yakaladığında etrafını yeşil bir dikdörtgen ile çevreliyoruz
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        faceDetect = True
#   Vucüt yakaladığında çıkaracağı sesleri ve motor hareketlerini sıralıyoruz.
    if GPIO.input(buttonPin):
        print(GPIO.input(buttonPin))
        if faceDetect:
            p1.ChangeDutyCycle(75)
            p2.ChangeDutyCycle(75)
            GPIO.output(in1,GPIO.HIGH)
            GPIO.output(in2,GPIO.LOW)
            GPIO.output(in3,GPIO.HIGH)
            GPIO.output(in4,GPIO.LOW)
            pygame.mixer.init()
            pygame.mixer.music.load("uyari.wav")
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy() == True:
                continue
            faceDetect = False
            time.sleep(7)
    
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0
    
#     Cihazın devrilme senaryosunu algılamasını ve gerekli sesleri çıkarmasını sağlıyoruz.
    if get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) < -30 or get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) > 30 or get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) < -30 or get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) > 30:
        print("*********************************************************")
        print("Cihaz DEVRİLDİ")
        print("*********************************************************")
        pygame.mixer.init()
        pygame.mixer.music.load("devrilme.wav")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy() == True:
                continue
      
    cv2.imshow("Frame", image)
    rawCapture.truncate(0) 

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()