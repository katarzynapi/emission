import sys
import new_grid as ng
import os
import json

def readJsonFile(filename):
    if os.path.isfile(filename):
        with open(filename) as json_file:
            data = json.load(json_file)
            return data
    else:
        return None
# END readJsonFile()

def isInt(value):
  try:
    int(value)
    return True
  except ValueError:
    return False
# END isInt()

def isFloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False
# END isFloat()

def validate_sys_argv(sys_argv):
    if len(sys.argv)<=1:
        print("Pass region as the 1st parameter. Currently only krk region can be used.")
        quit()
    elif len(sys.argv)<=2:
        if sys.argv[1] == 'init_file':
            print("Pass the name of init file as the 2nd parameter")
        else:
            print("Pass simulation start time in for hh:mm as the 2nd parameter")
        quit()
    elif len(sys.argv)<=3:
        if sys.argv[1] == 'init_file':
            return 1
        else:
            print("Pass interval duration in seconds/minutes/hours as the 3rd parameter")
        quit()
    elif len(sys.argv)<=4:
        print("Pass simulation duration in seconds/minutes/hours as the 4th parameter")
        quit()
    elif len(sys.argv)<=6:
        print("Pass segment size in meters as the 5th parameter")
        quit()
    elif len(sys.argv)<=6:
        print("Pass information about visualization (true/false) as 6th parameter")
        quit()
    return 1
#END validate_sys_argv()

def read_initial_data(sys_argv):
    initial_data_keys = ['region', 'start_time', 'interval_duration', 'simulation_duration', 'segment_size', 'visualisation']
    default_initial_data = ['krk', '00:00', '10s', '30s', '1000', 'false']
    if sys_argv[1] != 'init_file':
        initial_data = {k:arg for k, arg in zip(initial_data_keys, sys_argv[1:])}
        initial_data['start_time'] = initial_data['start_time'] if ng.checkTime(initial_data['start_time']) else None
        initial_data['interval_duration_unit'] = initial_data['interval_duration'][-1]
        initial_data['interval_duration'] = int(initial_data['interval_duration'][:-1])
        initial_data['simulation_duration_unit'] = initial_data['simulation_duration'][-1]
        initial_data['simulation_duration'] = int(initial_data['simulation_duration'][:-1])
        initial_data['segment_size'] = int(initial_data['segment_size'])
    else:
        filename = sys_argv[2]
        init_file = readJsonFile('input_data/'+filename)
        init_file = init_file['init_data']
        for k, d in zip(initial_data_keys, default_initial_data):
            if k not in init_file.keys():
                print(k + " not provided, default parameter (" + d + ") set.")
        initial_data_keys.extend(['interval_duration_unit', 'simulation_duration_unit'])
        initial_data = {k:arg for k, arg in zip(initial_data_keys, sys_argv[1:])}
        initial_data['region'] = init_file['region'] if 'region' in init_file.keys() else default_initial_data[0]
        initial_data['start_time'] = init_file['start_time'] if 'start_time' in init_file.keys() else default_initial_data[1]
        initial_data['interval_duration'] = init_file['interval_duration'][:-1] if 'interval_duration' in init_file.keys() else default_initial_data[2]
        initial_data['interval_duration_unit'] = init_file['interval_duration'][-1]
        initial_data['simulation_duration'] = init_file['simulation_duration'][:-1] if 'simulation_duration' in init_file.keys() else default_initial_data[3]
        initial_data['simulation_duration_unit'] = init_file['simulation_duration'][-1]
        initial_data['segment_size'] = int(init_file['segment_size']) if 'segment_size' in init_file.keys() else default_initial_data[4]
        initial_data['visualisation'] = init_file['visualisation'] if 'visualisation' in init_file.keys() else default_initial_data[5]

    return initial_data  
#END read_initial_data()
'''
region = sys.argv[1]
simulation_duration = int(sys.argv[2])
proper_start_time = ng.checkTime(sys.argv[3])
visualisation = sys.argv[4]
interval = int(sys.argv[5])

if region == 'grunwaldzkie':
# Rondo Grunwaldzkie
    min_longitude, max_longitude, min_latitude, max_latitude = 19.92810, 19.93787, 50.04730, 50.05112
elif region == 'aleje':
# I obwodnica Krakowa
    min_longitude, max_longitude, min_latitude, max_latitude = 19.9170, 19.9877, 50.0291, 50.0770
elif region == 'krk':
# Krakow
    min_longitude, max_longitude, min_latitude, max_latitude = 19.792236, 20.087346, 49.967667, 50.126134
else:
    print("Wrong region passed to script. Use grunwaldzkie, aleje or krk")
    quit()

if proper_start_time:
    start_time = sys.argv[3]
else:
    print("Wrong start time format. Use format hh:mm, where hh in <00,23> and mm in <00,59>")
    quit()

if interval > simulation_duration or simulation_duration%interval!=0:
    print("Passed interval is longer that simulation duration")
    quit()
if simulation_duration%interval != 0:
    print("Passed simulation duration is not an integer multiple of interval")
    quit()  
'''