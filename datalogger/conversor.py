#!/usr/bin/env python3
# coding: utf-8

filename = ''
n = 0
from time import sleep
from sys import argv
print(argv[0])							

while len(argv) >= 2:
	filename = argv[1]
	del argv[1]
	n = filename.find('.jpg')
	if n > 0:
		saveto = filename[0:n] + '_jpg.h'
	else:
		print('ERROR', filename)
		continue

	print('input filename  =',filename)
	print('output filename =',saveto)
	fp = open(filename, mode='br')
	data_array = fp.read()
	fp.close()
	out = []
	for d in data_array:
		out.append(d)

	fp = open(saveto, mode='w')
	print('#define '+filename[0:n]+'_jpg_len',len(out), file = fp)
	print('const uint8_t '+filename[0:n]+'_jpg[] = {', file = fp)
	i=0
	for d in out:
		# print(' ' + hex(d), end='', file = fp)
		print(' 0x' + format(d, '02X'), end='', file = fp)
		i += 1
		if i % 16 == 0:
			print(',', file = fp)
		else:
			if i < len(out):
				print(',', end='', file = fp)
	print('\n};', file = fp)
	fp.close()