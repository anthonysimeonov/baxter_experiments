#!/usr/bin/env python
import csv
import numpy as np

record_list = []

#initial filename, and joints we want to let move (ALWAYS INCLUDE TIME)
record_file = 'test_overwrite'
joint_keys = ['time', 'right_s1']

# Read all data from the csv file.
with open(record_file, 'rb') as b:
    read_in = csv.reader(b)
    record_list.extend(read_in)

#make numpy array
record_np = np.array(record_list[1:], dtype=float)

#create dictionary for choosing which joints to manipulate
unpack = []
for i in range(len(record_list[0])):
    unpack.append([record_list[0][i], i])

joint_dict = {key: value for (key, value) in unpack}

#get first value
initial_angles = record_list[1]
initial_angles_np = np.array(initial_angles, dtype=float)

#new file name
new_file = record_file + '_fixed'
# Write data to the csv file and replace the lines in the line_to_override dict.
with open(new_file, 'wb') as b:
    writer = csv.writer(b)
    writer.writerow(record_list[0])
    for i in range(record_np.shape[0]):
        #make list for overwriting
        line = record_np[i]
        line_in = initial_angles_np.copy()
        insert_inds = [joint_dict[x] for x in joint_keys]
        line_in[insert_inds] = line[insert_inds]
        line_in = list(np.array(line_in, dtype=str))
        writer.writerow(line_in)
