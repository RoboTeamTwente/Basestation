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

if ser.isOpen():
	print("Port opened!")

start = time.time()
last = start
cMsgs = 0
cBits = 0

received = []

while 1:
	out = ser.readline()
	cBits += len(out)
	cMsgs += 1
	now = time.time()
	# received.append(now)
	if 1 < now - last:
		mps = cMsgs / (now-start)
		mspm = 1000 * (now-start) / cMsgs
		kbps = 8 * (cBits/1000) / (now-start)
		print("%0.2f msgs/s    %0.2f ms/msg    %0.2f kbps" % (mps, mspm, kbps))
		last = now
	# if 0.05 < now - start:
	# 	break

# first = received[0]
# received = [1000*(r - first) for r in received]

# print("Plotting %d messages" % len(received))
# # seaborn histogram
# sns.distplot(received, hist=True, kde=False, 
#              bins=500, color = 'blue',
#              hist_kws={'edgecolor':'black'})
# # Add labels
# plt.title('Histogram of Message Arrival Times')
# plt.xlabel('Arrival (ms)')
# plt.ylabel('Messages')
# plt.tight_layout()
# plt.show()