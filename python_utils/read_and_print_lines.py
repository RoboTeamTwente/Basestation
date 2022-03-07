import utils

basestation = None

while True:
	# Open basestation with the basestation
	if basestation is None or not basestation.isOpen():
		basestation = utils.openContinuous(timeout=0.01)

	try:
		message = basestation.readline().decode()
		if len(message): print(message, end="")
	except:
		pass
