import matplotlib.pyplot as plt
import os
import numpy as np
import sys
import re
import heapq
from matplotlib.offsetbox import AnchoredText

Configtimestamps = []
XP, XI, XD, YP, YI, YD, WP, WI, WD, YawP, YawI, YawD, WheelsP, WheelsI, WheelsD = [], [], [], [], [], [], [], [], [], [], [], [], [], [], [] #PID gains

Statetimestamps = []
rateOfTurn = [] #estimated angular velocity
W1, W2, W3, W4 = [], [], [], [] #estimated wheel speeds
XsensX, XsensY, XsensYaw = [], [], [] #estimated x-, y accelerations and absolute angle
XInt, YInt, WInt, YawInt, W1Int, W2Int, W3Int, W4Int = [], [], [], [], [], [], [], [] #integral value from the I part of the PIDs

Feedbacktimestamps = []
estimatedAngle = []
Gamma, Phi = [], [] #estimated movement [m/s] and direction of movement [rad]
Vex, Vey = [], [] #estimated x- and y velocities calculated from gamma and phi

Commandedtimestamps = []
CommandedAngle, VisionAngle, AngleVel = [], [], [] #commanded angle and angular velocity
Rho, Theta = [], [] #commanded movement [m/s] and direction of movement [rad]
Vx, Vy = [], [] #commanded x- and y velocities calculated from rho and theta
Wc1, Wc2, Wc3, Wc4 = [], [], [], [] #commanded wheel speeds calculated from Vx and Vy

Visiontimestamps = []
visionVx, visionVy, visionYaw, visionW = [], [], [], []

plotsAvailable = ["x", "y", "w", "yaw", "wheels", "integral", "integral-wheels"]
# Parse input arguments 
try:
	if len(sys.argv) not in [2,3]:
		raise Exception("Error : Invalid number of arguments. Expected PID-index")
	plotID = sys.argv[1]
	if plotID not in plotsAvailable:
		raise Exception("Error : Unknown PID index %s. Choose one of the following : %s" % (plotID, ", ".join(plotsAvailable)))
except Exception as e:
	print(e)
	print("Error : Run script with \"python plotPID.py PID-index\"")
	exit()

lastfile = 0
if len(sys.argv) == 3: lastfile = int(sys.argv[2])
files = [file for file in os.listdir("PIDfiles") if file.startswith("robotStateInfo") and file.endswith(".csv")]
filenr = heapq.nlargest(lastfile+1, files)
filenr = re.findall('[0-9]+', str(filenr)) #find the time of the newest robotStateInfo file
CommandedLines = open("/home/elias/Downloads/test_data-20220503T150016Z-001/test data/yaw/vision files old PID values/2022-05-03_16-10-56_ROBOTCOMMANDS.txt", "r").read().strip().split("\n")
robotFeedbackLines = open("/home/elias/Downloads/test_data-20220503T150016Z-001/test data/yaw/vision files old PID values/2022-05-03_16-10-56_ROBOTFEEDBACK.txt", "r").read().strip().split("\n")
visionLines = open("/home/elias/Downloads/test_data-20220503T150016Z-001/test data/yaw/vision files old PID values/2022-05-03_16-10-58_BLUE_BOTS.txt", "r").read().strip().split("\n")

for i in range(len(CommandedLines)):
	if (i%2 == 0):
		Commandedtimestamps.append(CommandedLines[i][1:13])
		commas = []
		for j in range(len(CommandedLines[i])):
			if CommandedLines[i][j] == 'x':
				x = j+4
			if (CommandedLines[i][j] == 'y' and CommandedLines[i][j-1] == ' '):
				y = j+4
			if CommandedLines[i][j] == ',':
				commas.append(j)
			if CommandedLines[i][j-1] == 't' and CommandedLines[i][j] == 'A' and CommandedLines[i][j+1] == 'n' and CommandedLines[i][j+2] == 'g' and CommandedLines[i][j+3] == 'l':
				targetAngle = j + 7
			if CommandedLines[i][j-1] == 'l' and CommandedLines[i][j] == 'e' and CommandedLines[i][j+1] == ':':
				visAngle = j+2
		Vx.append(float(CommandedLines[i][x:commas[2]]))
		Vy.append(float(CommandedLines[i][y:commas[3]-2]))
		CommandedAngle.append(float(CommandedLines[i][targetAngle:commas[4]]))
		VisionAngle.append(float(CommandedLines[i][visAngle:commas[7]]))


for i in range(len(robotFeedbackLines)):
	if (i%2 == 0):
		Feedbacktimestamps.append(robotFeedbackLines[i][1:13])
		commas = []
		for j in range(len(robotFeedbackLines[i])):
			if robotFeedbackLines[i][j] == 'x' and robotFeedbackLines[i][j+1] == " ":
				velx = j+3
			if robotFeedbackLines[i][j] == 'y' and robotFeedbackLines[i][j+1] == " ":
				vely = j+3
			if robotFeedbackLines[i][j] == ',':
				commas.append(j)
			if robotFeedbackLines[i][j-1] == 'l' and robotFeedbackLines[i][j] == 'e' and robotFeedbackLines[i][j+1] == ':':
				estAngle = j+2
		rho = float(robotFeedbackLines[i][vely:commas[7]-2])
		theta = float(robotFeedbackLines[i][velx:commas[6]])
		curVx = float(rho)*np.cos(float(theta))
		curVy = float(rho)*np.sin(float(theta))
		Vex.append(-curVx)
		Vey.append(curVy)
		estimatedAngle.append(float(robotFeedbackLines[i][estAngle:commas[8]]))

for i in range(len(visionLines)):
	if (i%2 == 0):
		Visiontimestamps.append(visionLines[i][1:13])
		commas = []
		for j in range(len(visionLines[i])):
			if visionLines[i][j-2] == 'v' and visionLines[i][j-1] == "e" and visionLines[i][j] == "l":
				velx = j+4
			if visionLines[i][j-1] == "e" and visionLines[i][j] == ":":
				visAngle = j+1
			if visionLines[i][j] == ',':
				commas.append(j)
			if visionLines[i][j] == "w":
				w = j+2
			if visionLines[i][j] == "}":
				end = j-1
		visionVx.append(1.12*float(visionLines[i][velx:commas[1]]))
		visionVy.append(float(visionLines[i][commas[1]+1:commas[2]-1]))
		visionYaw.append(float(visionLines[i][visAngle:commas[5]]))
		visionW.append(float(visionLines[i][w:end]))
		
Commandedt0 = float(Commandedtimestamps[0][0:2])*60*60+float(Commandedtimestamps[0][3:5])*60 + float(Commandedtimestamps[0][6:8]) + float(Commandedtimestamps[0][9:12])/1000
for i in range(len(Commandedtimestamps)):
	Commandedtimestamps[i] = float(Commandedtimestamps[i][0:2])*60*60+float(Commandedtimestamps[i][3:5])*60 + float(Commandedtimestamps[i][6:8]) + float(Commandedtimestamps[i][9:12])/1000 - Commandedt0

for i in range(len(Feedbacktimestamps)):
	Feedbacktimestamps[i] = float(Feedbacktimestamps[i][0:2])*60*60+float(Feedbacktimestamps[i][3:5])*60 + float(Feedbacktimestamps[i][6:8]) + float(Feedbacktimestamps[i][9:12])/1000 - Commandedt0

for i in range(len(Visiontimestamps)):
	Visiontimestamps[i] = float(Visiontimestamps[i][0:2])*60*60+float(Visiontimestamps[i][3:5])*60 + float(Visiontimestamps[i][6:8]) + float(Visiontimestamps[i][9:12])/1000 - Commandedt0

if plotID == "bode":
	linesC = open('PIDfiles/velxCommand.csv', 'r').readlines()
	linesM = open('PIDfiles/velxMeasured.csv', 'r').readlines()
	tcom = [float(ln.replace('\n','').split(', ')[0]) for ln in linesC]
	vcom = [float(ln.replace('\n','').split(', ')[1]) for ln in linesC]
	tmeas = [float(ln.replace('\n','').split(', ')[0]) for ln in linesM]
	vmeas = [float(ln.replace('\n','').split(', ')[1]) for ln in linesM]

	fig, ax = plt.subplots()
	ax.plot(tcom, vcom)
	ax.plot(tmeas, vmeas)
	ax.set_title("Velocity in x-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()

if plotID == "x":
	fig, ax = plt.subplots()
	
	#ax.plot(Commandedtimestamps, Vx)
	ax.plot(Feedbacktimestamps, Vex)
	ax.plot(Visiontimestamps, visionVx)
	ax.set_title("Velocity in x-direction", fontsize = "xx-large")
	#ax.legend(["Commanded", "Estimated"])
	ax.legend(["Estimated", "Vision"])
	#ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (XP[0], ), r'$K_I=%.2f$' % (XI[0], ), r'$K_D=%.2f$' % (XD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax.add_artist(at)
	plt.show()
	
if plotID == "y":
	fig, ax = plt.subplots()
	
	#ax.plot(Commandedtimestamps, Vy)
	ax.plot(Feedbacktimestamps, Vey)
	ax.plot(Visiontimestamps, visionVy)
	ax.set_title("Velocity in y-direction", fontsize = "xx-large")
	#ax.legend(["Commanded", "Estimated"])
	ax.legend(["Estimated", "Vision"])
	#ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (YP[0], ), r'$K_I=%.2f$' % (YI[0], ), r'$K_D=%.2f$' % (YD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax.add_artist(at)
	plt.show()
	
if plotID == "w":
	fig, ax = plt.subplots()
	
	ax.plot(Feedbacktimestamps, AngleVel)
	ax.plot(Feedbacktimestamps, rateOfTurn)
	ax.set_title("Angular velocity", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	#ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [rad/s]", fontsize = "x-large")
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (WP[0], ), r'$K_I=%.2f$' % (WI[0], ), r'$K_D=%.2f$' % (WD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax.add_artist(at)
	plt.show()
	
if plotID == "yaw":
	fig, ax = plt.subplots()
	
	#ax.plot(Commandedtimestamps, CommandedAngle)
	ax.plot(Commandedtimestamps, VisionAngle)
	ax.plot(Feedbacktimestamps, estimatedAngle)
	ax.plot(Visiontimestamps, visionYaw)
	ax.set_title("Absolute angle", fontsize = "xx-large")
	ax.legend(["cameraAngle", "Estimated", "Vision"])
	#ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Angle [rad]", fontsize = "x-large")
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (YawP[0], ), r'$K_I=%.2f$' % (YawI[0], ), r'$K_D=%.2f$' % (YawD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax.add_artist(at)
	plt.show()

if plotID == "wheels":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Wheel velocities", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, Wc1)
	ax[0, 0].plot(Feedbacktimestamps, W1)
	ax[0, 0].set_title('RF')
	ax[0, 0].legend(["Commanded", "Estimated"])
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[0, 1].plot(Feedbacktimestamps, Wc2)
	ax[0, 1].plot(Feedbacktimestamps, W2)
	ax[0, 1].set_title('RB')
	ax[0, 1].legend(["Commanded", "Estimated"])
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 0].plot(Feedbacktimestamps, Wc3)
	ax[1, 0].plot(Feedbacktimestamps, W3)
	ax[1, 0].set_title('LB')
	ax[1, 0].legend(["Commanded", "Estimated"])
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 1].plot(Feedbacktimestamps, Wc4)
	ax[1, 1].plot(Feedbacktimestamps, W4)
	ax[1, 1].set_title('LF')
	ax[1, 1].legend(["Commanded", "Estimated"])
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (WheelsP[0], ), r'$K_I=%.2f$' % (WheelsI[0], ), r'$K_D=%.2f$' % (WheelsD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[1, 1].add_artist(at)
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Velocity [m/s]', va='center', rotation='vertical', fontsize = "x-large")

	plt.show()

if plotID == "integral":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Integral values from PIDs", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, XInt)
	ax[0, 0].set_title('X')
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (XP[0], ), r'$K_I=%.2f$' % (XI[0], ), r'$K_D=%.2f$' % (XD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[0, 0].add_artist(at)
	ax[0, 1].plot(Feedbacktimestamps, YInt)
	ax[0, 1].set_title('Y')
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (YP[0], ), r'$K_I=%.2f$' % (YI[0], ), r'$K_D=%.2f$' % (YD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[0, 1].add_artist(at)
	ax[1, 0].plot(Feedbacktimestamps, WInt)
	ax[1, 0].set_title('W')
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (WP[0], ), r'$K_I=%.2f$' % (WI[0], ), r'$K_D=%.2f$' % (WD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[1, 0].add_artist(at)
	ax[1, 1].plot(Feedbacktimestamps, YawInt)
	ax[1, 1].set_title('Yaw')
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (YawP[0], ), r'$K_I=%.2f$' % (YawI[0], ), r'$K_D=%.2f$' % (YawD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[1, 1].add_artist(at)
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Integral value', va='center', rotation='vertical', fontsize = "x-large")
	plt.show()
	
if plotID == "integral-wheels":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Integral values from wheel PIDs", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, W1Int)
	ax[0, 0].set_title('RF')
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#textstr = '\n'.join((r'$K_P=%.2f$' % (WheelsP[0], ), r'$K_I=%.2f$' % (WheelsI[0], ), r'$K_D=%.2f$' % (WheelsD[0], )))
	#at = AnchoredText(textstr, prop=dict(size=15), frameon=True, loc='center right')
	#at.patch.set_boxstyle("round,pad=0.,rounding_size=0.2")
	#ax[0, 0].add_artist(at)
	ax[0, 1].plot(Feedbacktimestamps, W2Int)
	ax[0, 1].set_title('RB')
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#ax[0, 1].add_artist(at)
	ax[1, 0].plot(Feedbacktimestamps, W3Int)
	ax[1, 0].set_title('LB')
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#ax[1, 0].add_artist(at)
	ax[1, 1].plot(Feedbacktimestamps, W4Int)
	ax[1, 1].set_title('LF')
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	# make box with PID gains
	#ax[1, 1].add_artist(at)
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Integral value', va='center', rotation='vertical', fontsize = "x-large")
	plt.show()
