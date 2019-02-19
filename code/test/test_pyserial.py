import time
import serial
  
ser = serial.Serial('/dev/ttyACM1', 9600)
num = 5

def send_data(msg):
	ser.write(msg)

while True:
    message = ser.readline()
    print(message.rstrip('\n'))
    if message[0] == 'C' :
        ser.write("ABCDE")
    time.sleep(0.5)

