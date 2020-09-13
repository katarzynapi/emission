import osmnx as ox
import networkx as nx
import math
import matplotlib.pyplot as plt
import pickle
import sys
import new_grid as ng
import new_traffic as nt
import init

if init.validate_sys_argv(sys.argv) == 1:
    pass
    print("Proper initial parameters provided.")
else:
    print("Wrong parameters provided within the command.")
    quit()

initial_data = init.read_initial_data(sys.argv)

if initial_data['region'] == 'grunwaldzkie':
# Rondo Grunwaldzkie
    min_longitude, max_longitude, min_latitude, max_latitude = 19.92810, 19.93787, 50.04730, 50.05112
elif initial_data['region'] == 'aleje':
# I obwodnica Krakowa
    min_longitude, max_longitude, min_latitude, max_latitude = 19.9170, 19.9877, 50.0291, 50.0770
elif initial_data['region'] == 'krk':
# Krakow
    min_longitude, max_longitude, min_latitude, max_latitude = 19.792236, 20.087346, 49.967667, 50.126134
else:
    print("Wrong region passed to script. Use grunwaldzkie, aleje or krk")
    quit()

if initial_data['start_time'] == None:
    print("Wrong start time format. Use format hh:mm, where hh in <00,23> and mm in <00,59>")
    quit()

if not init.isInt(initial_data['interval_duration']) or not init.isFloat(initial_data['interval_duration']):
    print("Passed interval duration is not a numeric value.")
    quit()
else:
    initial_data['interval_duration'] = int(initial_data['interval_duration'])
    
if not init.isInt(initial_data['simulation_duration']) or not init.isFloat(initial_data['simulation_duration']):
    print("Passed simulation duration is not a numeric value.")
    quit()
else:
    initial_data['simulation_duration'] = int(initial_data['simulation_duration'])

if initial_data['interval_duration_unit'] not in ['s', 'm', 'h']:
    print("Wrong interval duration unit. Use s (seconds), m (minutes) or h (hours)")
    quit()
else:
    if initial_data['interval_duration_unit'] == 'm':
        initial_data['interval_duration'] *= 60
    elif initial_data['interval_duration_unit'] == 'h':
        initial_data['interval_duration'] *= 3600
    elif initial_data['interval_duration_unit'] == 's':
        pass
    else:
        print("Wrong interval duration unit. Use s (seconds), m (minutes) or h (hours)")
        quit()

if initial_data['simulation_duration_unit'] not in ['s', 'm', 'h']:
    print("Wrong simulation duration unit. Use s (seconds), m (minutes) or h (hours)")
    quit()
else:
    if initial_data['simulation_duration_unit'] =='m':
        initial_data['simulation_duration'] *= 60
    elif initial_data['simulation_duration_unit'] == 'h':
        initial_data['simulation_duration'] *= 3600
    elif initial_data['simulation_duration_unit'] == 's':
        pass
    else:
        print("Wrong simulation duration unit. Use s (seconds), m (minutes) or h (hours)")
        quit()

if int(initial_data['interval_duration']) > initial_data['simulation_duration']:
    print("Passed interval is longer that simulation duration")
    quit()
if initial_data['simulation_duration']%initial_data['interval_duration'] != 0:
    print("Passed simulation duration is not an integer multiple of interval")
    quit()  

G_filename = "G_" + initial_data['region'] + ".p"
my_network_filename = "my_network_" + initial_data['region'] + ".p"

print("Arguments passed")

#Simplification speeds up all calculation for the prise of accuracy
simplify_map = False
#Network can be 'all' or 'drive', all is slower
network_map = 'drive'

# Create Graph
#G_whole = ox.graph_from_bbox(max_latitude, min_latitude, max_longitude, min_longitude, network_type=network_map, simplify=simplify_map)
#print("Whole graph created")
#pickle.dump(G_whole, open(G_filename, "wb"))
#print("Whole graph serialized")

# Or read graph from file (deserialize)
G_whole = pickle.load( open( G_filename, "rb" ) )
print("Whole graph deserialized")
# Reduce map
prio_highway_tags = ['motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link']
G_reduced = ng.reduceMap(G_whole, prio_highway_tags)
print("Graph reduced")

# Create road network for reduced map
my_network = ng.RoadNetwork(G_reduced, min_longitude, max_longitude, min_latitude, max_latitude,\
    input_nodes=[1780711571, 30372002, 206374756, 243362266, 3303001786, 247096831], output_nodes=[1780711571, 30372002, 206374756, 243362266, 3303001786],\
    segment_size = initial_data['segment_size'])
print("Road network for reduced graph created")
# Save road network to file (serialization)
pickle.dump(my_network, open(my_network_filename, "wb"))
print("Road network for reduced graph serialized")
# Show road network on the map
my_network = pickle.load( open( my_network_filename, "rb" ) )
print("Road network for reduced graph deserialized")

#my_network.showRoadNetwork(inputs=True, outputs=True, lights=True, segments=True)
#my_network.showRoadNetwork(segments=True)
#my_network.showRoadNetwork(inputs=True, outputs=True)
#my_network.showRoadNetwork(cells=True)

# Create simulation model

model = nt.TrafficModel(my_network, initial_data['simulation_duration'], initial_data['start_time'], initial_data['interval_duration'])
print("Simulation model created")
if initial_data['visualisation']=="true":
    model.showMultipleAgentsMovement()
else:
    model.simulateMultipleAgentsMovement()
model.showMaxSegmentPollutionDistribution("CO", 3)
print("Summary Emission:")
print(model.getSummaryEmission("CO"))