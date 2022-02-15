import os
import datetime

with open("RoveCommManifest.h") as f:
	lines = f.readlines()
	
output = open("RoveCommManifest.py", "w+")
output.write("#Compiled at " + str(datetime.datetime.now()) + "\n" + 
				"uint8_t = 'B'\n" + 
				"int8_t = 'b'\n" + 
				"uint16_t = 'H'\n" + 
				"int16_t = 'h'\n" + 
				"uint32_t = 'L'\n" + 
				"int32_t = 'l'\n" + 
				"uint64_t = 'Q'\n" + 
				"int64_t = 'q'\n"
				)
lines = lines[2:]
lines = lines[:-1]
	
for line in lines:
	line = line.replace("	", " ")
	if "#define" in line:
		line = line.replace("#define ", "", 1).replace(" ", "=", 1)
			
	
	line = line.replace(" 01", " 1")
	line = line.replace(" 02", " 2")
	line = line.replace(" 03", " 3")
	line = line.replace(" 04", " 4")
	line = line.replace(" 05", " 5")
	line = line.replace(" 06", " 6")
	line = line.replace(" 07", " 7")
	line = line.replace(" 08", " 8")
	line = line.replace(" 09", " 9")

	line = line.replace("//", "#", 1).replace("/*", "'''").replace("*/", "'''")
	output.write(line)