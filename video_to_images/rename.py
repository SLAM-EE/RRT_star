from shutil import copyfile
import os
loc = os.getcwd() + '/images'
for i, filename in enumerate(os.listdir(loc)):
	l = len(filename)
	ss = '0' * (10-l) + filename
	loc_o = loc + '/' + filename
	os.rename(loc_o, loc + '/' + str(ss))
copyfile(loc + '/' +'000001.png', loc + '/' +'000000.png')
