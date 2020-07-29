import subprocess
import sys

for i in range(len(sys.argv)):
	if sys.argv[i] == "-A":
		algorithm = sys.argv[i+1]
	elif sys.argv[i] == "-v":
		vehicle = sys.argv[i+1]
	elif sys.argv[i] == "-m":
		grid = sys.argv[i+1]
	elif sys.argv[i] == "-t":
		time = sys.argv[i+1]
	elif sys.argv[i] == "-n":
		iterations = sys.argv[i+1]
	elif sys.argv[i] == "-s":
		start = sys.argv[i+1]
	elif sys.argv[i] == "-g":
		goal = sys.argv[i+1]

print "0 0 0 0"
for i in range(int(iterations)):
	args = ("./"+algorithm, "-v", vehicle, "-m", grid, "-t", time, "-s", start, "-g", goal)
	popen = subprocess.Popen(args, stdout=subprocess.PIPE)
	popen.wait()
	output = popen.stdout.read()
	print output
	print "0 0 0 0"