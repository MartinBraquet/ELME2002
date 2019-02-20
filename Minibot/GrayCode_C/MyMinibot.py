import RPi.GPIO as GPIO
from time import sleep
import spidev

MyARM_ResetPin = 19 # Pin 4 of connector = BCM19 = GPIO[1]

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,0)
MySPI_FPGA.max_speed_hz = 500000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MyARM_ResetPin, GPIO.OUT)

GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(0.1)

ToSPI = [0x01, 0x00, 0x00, 0x00, 0x00]
FromSPI = MySPI_FPGA.xfer2(ToSPI)

print(FromSPI)

GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)


