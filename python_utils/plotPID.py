import matplotlib.pyplot as plt
import os
import numpy as np
import sys
import re
from matplotlib.offsetbox import AnchoredText

files = [file for file in os.listdir("PIDfiles") if file.startswith("robotStateInfo") and file.endswith(".csv")]
latest_file = re.findall('[0-9]+', max(files)) #find the time of the newest robotStateInfo file

#ConfigLines = open("robotConfig_1643811941.csv", "r").read().strip().split("\n")
StateInfoLines = open("PIDfiles/robotStateInfo_" + latest_file[0] + ".csv", "r").read().strip().split("\n")
CommandedLines = open("PIDfiles/robotCommand_" + latest_file[0] +  ".csv", "r").read().strip().split("\n")
robotFeedbackLines = open("PIDfiles/robotFeedback_" + latest_file[0] + ".csv", "r").read().strip().split("\n")

Configtimestamps = []
XP, XI, XD, YP, YI, YD, WP, WI, WD, YawP, YawI, YawD, WheelsP, WheelsI, WheelsD = [], [], [], [], [], [], [], [], [], [], [], [], [], [], [] #PID gains

Statetimestamps = []
rateOfTurn = [] #estimated angular velocity
W1, W2, W3, W4 = [], [], [], [] #estimated wheel speeds
XsensX, XsensY, XsensYaw = [], [], [] #estimated x-, y accelerations and absolute angle
XInt, YInt, WInt, YawInt, W1Int, W2Int, W3Int, W4Int = [], [], [], [], [], [], [], [] #integral value from the I part of the PIDs

Feedbacktimestamps = []
Gamma, Phi = [], [] #estimated movement [m/s] and direction of movement [rad]
Vex, Vey = [], [] #estimated x- and y velocities calculated from gamma and phi

Commandedtimestamps = []
Angle, AngleVel = [], [] #commanded angle and angular velocity
Rho, Theta = [], [] #commanded movement [m/s] and direction of movement [rad]
Vx, Vy = [], [] #commanded x- and y velocities calculated from rho and theta
Wc1, Wc2, Wc3, Wc4 = [], [], [], [] #commanded wheel speeds calculated from Vx and Vy

plotsAvailable = ["x", "y", "w", "yaw", "wheels", "integral", "integral-wheels"]

# Parse input arguments 
try:
	if len(sys.argv) != 2:
		raise Exception("Error : Invalid number of arguments. Expected PID-index")
	plotID = sys.argv[1]
	if plotID not in plotsAvailable:
		raise Exception("Error : Unknown PID index %s. Choose one of the following : %s" % (plotID, ", ".join(plotsAvailable)))
except Exception as e:
	print(e)
	print("Error : Run script with \"python plotPID.py PID-index\"")
	exit()


#for line in ConfigLines[100:500]:
#	ts, xp, xi, xd, yp, yi, yd, wp, wi, wd, yawp, yawi, yawd, wheelsp, wheelsi, wheelsd = line.split(" ")
#	ts = int(float(ts) * 1000)
#	Configtimestamps.append(ts)
#	XP.append(float(xp))
#	XI.append(float(xi))
#	XD.append(float(xd))
#	YP.append(float(yp))
#	YI.append(float(yi))
#	YD.append(float(yd))
#	WP.append(float(wp))
#	WI.append(float(wi))
#	WD.append(float(wd))
#	YawP.append(float(yawp))
#	YawI.append(float(yawi))
#	YawD.append(float(yawd))
#	WheelsP.append(float(wheelsp))
#	WheelsI.append(float(wheelsi))
#	WheelsD.append(float(wheelsd))

for line in StateInfoLines:
	ts, xsensx, xsensy, xsensyaw, rot, w1, w2, w3, w4, xint, yint, wint, yawint = line.split(" ")[:13]#, w1int, w2int, w3int, w4int = line.split(" ")[:17]
	ts = int(float(ts) * 1000)
	Statetimestamps.append(ts)
	
	XsensX.append(float(xsensx))
	XsensY.append(float(xsensy))
	XsensYaw.append(float(xsensyaw))
	rateOfTurn.append(float(rot))
	W1.append(float(w1))
	W2.append(float(w2))
	W3.append(float(w3))
	W4.append(float(w4))
	XInt.append(float(xint))
	YInt.append(float(yint))
	WInt.append(float(wint))
	YawInt.append(float(yawint))
	#W1Int.append(float(w1int))
	#W2Int.append(float(w2int))
	#W3Int.append(float(w3int))
	#W4Int.append(float(w4int))
	
	
for line in CommandedLines: 
	ts, ang, angvel, rho, theta = line.split(" ")[:5]
	ts = int(float(ts) * 1000)
	Commandedtimestamps.append(ts)
	
	Angle.append(float(ang))
	AngleVel.append(float(angvel))
	Rho.append(float(rho))
	Theta.append(float(theta))
	curVx = float(rho)*np.cos(float(theta))
	curVy = float(rho)*np.sin(float(theta))
	Vx.append(curVx)
	Vy.append(curVy)
	Wc1.append((curVx * np.cos(30 * np.pi/180) + curVy * np.sin(30 * np.pi/180))/0.028  + float(angvel) * 0.081 / 0.028) #equations from body2wheels in stateControl.c
	Wc2.append((curVx * np.cos(60 * np.pi/180) + curVy * -np.sin(60 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028) 
	Wc3.append((curVx * -np.cos(60 * np.pi/180) + curVy * -np.sin(60 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028)  
	Wc4.append((curVx * -np.cos(30 * np.pi/180) + curVy * np.sin(30 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028) 

for line in robotFeedbackLines:
	ts, gamma, phi = line.split(" ")[:3]
	ts = int(float(ts) * 1000)/1000 #multiply by 1000 to get ms accuracy, divide by 1000 to get x-axis in seconds
	Feedbacktimestamps.append(ts)
	
	Gamma.append(float(gamma))
	Phi.append(float(phi))
	curVex = float(gamma)*np.cos(float(phi))
	curVey = float(gamma)*np.sin(float(phi))
	Vex.append(-curVex)
	Vey.append(curVey)

#Shift the time values to make it start at 0
firstTimeVal = Feedbacktimestamps[0]
for i in range(len(Feedbacktimestamps)):
	Feedbacktimestamps[i] -= firstTimeVal

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
	
	ax.plot(Feedbacktimestamps, Vx)
	ax.plot(Feedbacktimestamps, Vex)
	ax.set_title("Velocity in x-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
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
	
	ax.plot(Feedbacktimestamps, Vy)
	ax.plot(Feedbacktimestamps, Vey)
	ax.set_title("Velocity in y-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
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
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
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
	
	ax.plot(Feedbacktimestamps, Angle)
	ax.plot(Feedbacktimestamps, XsensYaw)
	ax.set_title("Absolute angle", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
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
