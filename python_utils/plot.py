import matplotlib.pyplot as plt
import os
files = [file for file in os.listdir() if file.startswith("robotStateInfo") and file.endswith(".csv")]


# lines = open("robotStateInfo_1632500763.csv", "r").read().strip().split("\n")
# lines = open("robotStateInfo_1632501235.csv", "r").read().strip().split("\n")
lines = open("robotStateInfo_1643811941.csv", "r").read().strip().split("\n")


timestamps = []
rateOfTurn = []
W1, W2, W3, W4 = [], [], [], []

for line in lines[100:500]:
	ts, rot, w1, w2, w3, w4 = line.split(" ")
	ts = int(float(ts) * 1000)
	rot = float(rot)
	timestamps.append(ts)
	rateOfTurn.append(rot)

	W1.append(float(w1))
	W2.append(float(w2))
	W3.append(float(w3))
	W4.append(float(w4))


plt.plot(timestamps, rateOfTurn)
#plt.plot(timestamps, Is)
#plt.legend([])
plt.show()

