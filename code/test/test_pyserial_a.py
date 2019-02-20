import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
time.sleep(2)

while 1:
    
    command = str.encode('8')
    arduino.write(command)   
    time.sleep(1.5)
