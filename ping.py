import time
import serial
import matplotlib.pyplot as plt
import seaborn as sns

ser = serial.Serial(
    port='/dev/ttyACM1',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

if not ser.isOpen():
	print("Could not open %s!" % ser.port)
	exit()

string = "123456789012345678901234567890123456789012345678901234567890\n"
while True:

	ser.write(string.encode())
	while ser.inWaiting():
		print(ser.readline().decode("utf-8"), end="")
	time.sleep(0.2)