import subprocess
import sys

for i in range(len(sys.argv)):
	if sys.argv[i] == "-A":
		algorithm = sys.argv[i+1]
	elif sys.argv[i] == "-v":
		vehicle = sys.argv[i+1]
	elif sys.argv[i] == "-t":
		time = sys.argv[i+1]
	elif sys.argv[i] == "-f":
		filename = sys.argv[i+1]

file = open(filename,"r") 
line = file.readlines()

print "0 0 0 0"
for i in range(len(line)):
	line_comps = line[i].split()
	if int(line_comps[0]) >= 50 and int(line_comps[0]) < 70:
		W = int(line_comps[2])
		L = int(line_comps[3])
		s_x = int(line_comps[4])
		s_y = int(line_comps[5])
		g_x = int(line_comps[6])
		g_y = int(line_comps[7])
		s_N = str((L-s_y)*W+s_x+1)
		g_N = str((L-g_y)*W+g_x+1)
		
		args = ("./"+algorithm, "-v", vehicle, "-m", "./maps/"+line_comps[1], "-t", time, "-s", s_N, "-g", g_N)
		popen = subprocess.Popen(args, stdout=subprocess.PIPE)
		popen.wait()
		output = popen.stdout.read()
		print output
		print line_comps[0],"0 0 0"

file.close()