'''
Katarzyna Piwowarczyk
funkcje i klasy do generowania mapy miasta z nałożonymi:
- wejściami i wyjściami,
- ulicami (+dyskretyzacja),
- światłami,
- siatką do zliczania zanieczyszczeń
'''


import osmnx as ox
import networkx as nx
import geopy.distance as gd
import math
#Matplotlib for visualisation
import matplotlib.pyplot as plt
import enums
import time
import json
import os
import numpy as np

def reduceMap(graph, highway_tags_to_remain):
    '''
    DESCRIPTION:
    reduce graph to edges with highway_tags_to_remain
    INPUT:
        graph - the whole graph to be reduced,
        highway_tags_to_remain - list of highway tags, edges with those tags remain in graph
    OUTPUT: reduced graph, only with edges with tags in highway_tags_to_remain
    '''
    # copy graph
    graph_reduced = graph.copy()
    street_not_to_remove = ["Armii Krajowej", "Jasnogórska", "Nawojki", "Czarnowiejska", "Mirowska", "Księcia Józefa", "Tadeusza Kościuszki", "Rondo Herberta Hoovera"]
    # remove edges
    for i,j in graph.edges():
        if 'name' in graph_reduced[i][j][0].keys():
            if graph_reduced[i][j][0]['name'] in street_not_to_remove and graph_reduced[i][j][0]['highway'] in ["secondary", "secondary_link"]:
                continue
        if graph_reduced[i][j][0]['highway'] not in highway_tags_to_remain:
            graph_reduced.remove_edge(i,j)
    # create a set of remained edges
    edges_begin = [e[0] for e in graph_reduced.edges()]
    edges_end = [e[1] for e in graph_reduced.edges()]
    edges_begin.extend(edges_end)
    edges = set(edges_begin)
    # remove remaining nodes, not assigned to any edge
    nodes_to_remove = [n for n in graph_reduced.nodes if n not in edges]
    graph_reduced.remove_nodes_from(nodes_to_remove)
    return graph_reduced
# END reduceMap()
    
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

def checkTime(time):
    time_split = time.split(":")
    if len(time_split) == 2 and isInt(time_split[0]) and isInt(time_split[1]) and\
        int(time_split[0])<=23 and int(time_split[0])>=0 and int(time_split[1])<=59 and int(time_split[1])>=0:
        return True
    else:
        return False
# END checkTime()
#__________________________________________________________________________________________________
class Coordinates():
    def __init__(self, x_coords=0.0, y_coords=0.0):
        self.x = x_coords
        self.y = y_coords

    def calculateDistance(self, coords):
        return gd.geodesic((self.y, self.x), (coords.y, coords.x)).m
#END class Coordinates()

#__________________________________________________________________________________________________
class Cell():
    def __init__(self, maxspeed, x_coords=0.0, y_coords=0.0, segment=None):
        self.coords = Coordinates(x_coords, y_coords)
        self.agent = None                 # założyłam, że będzie jeden agent na jednym polu
        self.lights = []
        self.pollution = {"NO":0, "PM":0, "CO":0, "HC":0} #jednostka to gram
        self.segment = segment
        self.connected_path = None
        self.maxspeed = maxspeed
    
    def getCoords(self):
        return self.coords.x, self.coords.y
    
    def addLights(self, lights):
        self.lights.append(lights)
#END class Cell()

#__________________________________________________________________________________________________
class Path():
    def __init__(self, cells, probability=1):
        self.cells = cells
        #self.probability = probability
        self.type = None
#END class Path()

#__________________________________________________________________________________________________
class Light():
    def __init__(self, location, connected_paths, ltype=enums.LightType.NULL):
        self.location = location            # typ to Cell (nie Coordinates)
        self.visibility = 0.0
        self.agent_types = []
        self.speed_limit = 0.0
        self.connected_paths = []
        self.colour = enums.Colour.RED
        self.type = self.setType(ltype)
    
    def setType(self, ltype):
        elt = enums.LightType
        string_ltypes = {'n':elt.NORTH, 'e':elt.EAST, 's':elt.SOUTH, 'w':elt.WEST}
        if ltype in elt:
            return ltype
        elif ltype in string_ltypes.keys():
            return string_ltypes[ltype]
        else:
            return enums.LightType.NULL

    def changeColour(self):
        self.colour = enums.Colour.GREEN if self.colour == enums.Colour.RED else enums.Colour.RED
#END class Light()

#__________________________________________________________________________________________________
class RoadNetwork():
    def __init__(self, G, min_lon, max_lon, min_lat, max_lat, inputs_probability=None, rate=100000, segment_size=1000, input_nodes=[], output_nodes=[]):
        t = time.time()
        self.G = G
        self.min_lon, self.max_lon, self.min_lat, self.max_lat = min_lon, max_lon, min_lat, max_lat
        self.lane_keys = ["first", "second", "third"]
        self.edges = self.initiateEdges()
        print('Edges initiated.')
        self.inputs, self.outputs = self.findInputOutputNodes()
        print('Input and output nodes found.')
        self.addSelfDefinedInputNodes(input_nodes)
        self.addSelfDefinedOutputNodes(output_nodes)
        print('Self-defined input and output nodes added.')
        self.calculateRoadCells()
        print('Road network discretised.')
        #self.addLanes()
        #print('Lanes added')
        self.lights_direction = self.getLightsDirections()
        print('Lights direction data initialised.')
        self.lights, self.light_nodes = self.assignLights()
        print('Lights assigned to the road network.')
        self.inputs_probability = self.assignInputProbabilities() # dictionary with hour(as numbers):list of nodes with probabilities
        self.outputs_probability = self.assignOutputProbabilities() # list with probabilities corresponding to list self.outputs
        print('Input and output probabilites assigned.')
        self.paths, self.path_keys = self.generatePaths()
        print('Paths generated.')
        self.lon_segments_no = None
        self.lat_segments_no = None
        self.segment_size = segment_size
        self.lon_segment_borders, self.lat_segment_borders, self.segments = self.generatePollutionSegments()
        print('Map segments generated.')
        self.assignSegmentToCells()
        print('Segments assigned to cells.')
        
    def calculateDistance(self, lon1, lat1, lon2, lat2):
        '''
        DESCRIPTION:
        calculates distance between two points on the map
        INPUT: lon1, lat1 - longitute and latitude of the 1st point, lon2, lat2 - longitute and latitude of the 2nd point
        OUTPUT: - distance in between points in [m]
        '''
        return gd.geodesic((lat1, lon1), (lat2, lon2)).m
        # END calculateDistance()
        
    def calculateDistanceDivider(self, distance):
        '''
        DESCRIPTION:
        calculates distance divider between two points on the map
        INPUT: distance - distance between two points on the map
        OUTPUT: - divider - number of segments between two points of the map for the purpose of discretization
        '''
        if distance <= 1.25:
            divider = 1
        elif distance > 1.25 and distance <= 2:
            divider = 2
        elif distance > 2:
            floor  = math.floor(distance)
            mant = distance - floor
            divider = floor if mant < 0.5 else floor+1
        else:
            divider = None
        return divider
        # END calculateDistanceDivider()
        
    def addTwoValues(self, a1, a2):
        '''
        DESCRIPTION:
        calculates a sum of two values
        INPUT: a1, a2 - numerical values to be added
        OUTPUT: sum of a1 and a2
        '''
        return a1+a2
        # END addTwoValues()
        
    def substractTwoValues(self, a1, a2):
        '''
        DESCRIPTION:
        calculates a substraction of two values
        INPUT: a1, a2 - numerical values to be substracted
        OUTPUT: substraction of a1 and a2
        '''
        return a1-a2
        # END substractTwoValues()
    
    def calculateDiscretePoints(self, lon1, lat1, lon2, lat2, distanceDivider):
        '''
        DESCRIPTION:
        calculates middle-points on the line between two points on the map, points are evenly spaced with distance circa 1[m]
        INPUT: lon1, lat1 - longitute and latitude of the 1st point, lon2, lat2 - longitute and latitude of the 2nd point
                distanceDivider - number of segments between two points of the map for the purpose of discretization
        OUTPUT: list of middle-points, including starting and ending point
        '''
        edgePoints = []
        edgePoints.append((lon1, lat1))
        
        lon_distance = math.fabs(lon2 - lon1)
        lon_step = lon_distance / distanceDivider
        lonStepFunction = self.addTwoValues if lon2>lon1 else self.substractTwoValues
        
        lat_distance = math.fabs(lat2 - lat1)
        lat_step = lat_distance / distanceDivider
        latStepFunction = self.addTwoValues if lat2>lat1 else self.substractTwoValues
         
        for step in range(distanceDivider-1):
            step_lon = lonStepFunction(edgePoints[-1][0], lon_step)
            step_lat = latStepFunction(edgePoints[-1][1], lat_step)
            edgePoints.append((step_lon, step_lat))
        
        edgePoints.append((lon2, lat2))
        return edgePoints
        # END calculateDiscretePoints()
        
    def discretizeEdge(self, lon1, lat1, lon2, lat2):
        '''
        DESCRIPTION:
        the whole process of finding middle-points between two points on the map, calculated points are evenly spaced with distance circa 1[m]
        INPUT: lon1, lat1 - longitute and latitude of the 1st point, lon2, lat2 - longitute and latitude of the 2nd point
        OUTPUT: list of middle-points, including starting and ending point
        '''
        distance = self.calculateDistance(lon1, lat1, lon2, lat2)
        distanceDivider = self.calculateDistanceDivider(distance)
        edgePoints = self.calculateDiscretePoints(lon1, lat1, lon2, lat2, distanceDivider)
        return edgePoints
        # END discretizeEdge()
        
    def initiateEdges(self):
        '''
        DESCRIPTION:
        initiate dictionary for storing middle-points within all edges, including number of lanes
        INPUT: -
        OUTPUT: edges - dictionary with keys as a tuple (start_road_node, end_road_node)
                ( start_road_node and end_road_node are node ids )
                and values as dictionary with keys "first", "second", "third" representing consecutive lanes and values as a list of points (lon, lat) between nodes
        '''
        edges = {}
        for start_road_node, end_road_node in self.G.edges():
            edges[start_road_node, end_road_node] = {"first":None, "second":None, "third":None}
        return edges
        # END initiateEdges()
        
    def getNewStartEndCoords(self, direction, start_coords, end_coords):
        '''
        DESCRIPTION:
        INPUT:
            direction - direction of path (right, down, left, up)
            start_coords, end_coords - coords of starting and ending point of basic lane
        OUTPUT: coords of starting and ending point of new lane
        '''
        lon_start, lat_start = start_coords
        lon_end, lat_end = end_coords
        if direction == "right":
            lon_start -= 0.00005
            lon_end -= 0.00005
        elif direction == "down":
            lat_start -= 0.00005
            lat_end -= 0.00005
        elif direction == "left":
            lon_start += 0.00005
            lon_end += 0.00005
        elif direction == "up":
            lat_start += 0.00005
            lat_end += 0.00005
        else:
            print("Wrong direction in getNewStartEndCoords()")
            return None
        return (lon_start, lat_start), (lon_end, lat_end)
        # END getNewStartEndCoords()
        
    def addLanes(self):
        '''
        DESCRIPTION:
        INPUT: -
        OUTPUT: -
        '''
        for start_road_node, end_road_node in self.G.edges():
            pi = 3.14
            edge = self.G[start_road_node][end_road_node][0]
            # get lanes and direction parameters
            if "lanes" in edge.keys():
                lanes_no = int(edge["lanes"])
            else:
                lanes_no = 1
            # get coords for nodes needed for edge angle (direction of road) calculation
            start_road_node_coords = self.G.node[start_road_node]['x'], self.G.node[start_road_node]['y']
            end_road_node_coords = self.G.node[end_road_node]['x'], self.G.node[end_road_node]['y']
            north_road_node_coords = self.G.node[start_road_node]['x'], 1
            main_vector_N = north_road_node_coords[0]-start_road_node_coords[0], north_road_node_coords[1]-start_road_node_coords[1]
            road_vector = end_road_node_coords[0]-start_road_node_coords[0], end_road_node_coords[1]-start_road_node_coords[1]
            angle = math.atan2(main_vector_N[1], main_vector_N[0]) - math.atan2(road_vector[1], road_vector[0])
            # determined angle between edge vector and S-N vector
            if angle < 0:
                angle += 2 * pi
            if (angle >= 0 and angle < pi/4) or (angle <= 2*pi and angle >= 7*pi/4):
                direction = "right"
            elif angle >= pi/4 and angle < 3*pi/4:
                direction = "down"
            elif angle >= 3*pi/4 and angle < 5*pi/4:
                direction = "left"
            elif angle >= 5*pi/4 and angle < 7*pi/4:
                direction = "up"
            else:
                print("Incorrect angle value in add_lanes()", angle)
                print(main_vector_N, road_vector)
            # add new lanes according to above-calculated parameters
            if lanes_no == 2 or lanes_no == 3:
                path_cells = self.edges[start_road_node, end_road_node]["first"].cells
                start_coords = path_cells[0].getCoords()
                end_coords = path_cells[-1].getCoords()
                new_start_coords, new_end_coords = self.getNewStartEndCoords(direction, start_coords, end_coords)
                self.generatePath(start_road_node, end_road_node, new_start_coords, new_end_coords, "second")
                if lanes_no == 3:
                    new_start_coords, new_end_coords = self.getNewStartEndCoords(direction, new_start_coords, new_end_coords)
                    self.generatePath(start_road_node, end_road_node, new_start_coords, new_end_coords, "third")
        # END addLanes()

    def generatePath(self, start_road_node, end_road_node, start_road_node_coords, end_road_node_coords, lane):
        '''
        DESCRIPTION:
        INPUT:
        OUTPUT:
        '''
        #calculate cells coords for selected edge
        # TO DO
        middle_points = self.discretizeEdge(start_road_node_coords[0], start_road_node_coords[1], end_road_node_coords[0], end_road_node_coords[1])
        edge = self.G[start_road_node][end_road_node][0]
        if "maxspeed" in edge.keys():
            maxspeed = edge["maxspeed"]
        else:
            maxspeed = "50"
        new_path = Path([Cell(maxspeed, lon, lat) for lon, lat in middle_points])
        #assign path reference to all cells
        for c in new_path.cells:
            c.connected_path = new_path
        self.edges[start_road_node, end_road_node][lane] = new_path
        # END generatePath()
        
    def calculateRoadCells(self):
        '''
        DESCRIPTION:
        calculate cells for all edges in the graph and store them in dictionary
        keys -> tuples of (node1, node2) representing edge
        values -> lists of (lon, lat) representing consecutive cells within edge
        INPUT:
        OUTPUT:
        '''
        for start_road_node, end_road_node in self.G.edges():
            #get start and end node coords for selected edge
            start_road_node_coords = self.G.node[start_road_node]['x'], self.G.node[start_road_node]['y']
            end_road_node_coords = self.G.node[end_road_node]['x'], self.G.node[end_road_node]['y']
            self.generatePath(start_road_node, end_road_node, start_road_node_coords, end_road_node_coords, "first")
        # END calculateRoadCells()

    def getLightsDirections(self):
        '''
        DESCRIPTION: reads json file with lights directions and return the dictionary
        INPUT: -
        OUTPUT: dictionary with pairs: (key:node, value:direction)
        '''
        filename = "input_data/lights_direction.json"
        lights_direction = readJsonFile(filename)
        lights_direction_final = lights_direction["lights"]
        return lights_direction_final
        # END getLightsDirections()
    
    def assignLights(self):
        '''
        DESCRIPTION: assign lights to path cell representing start node
        INPUT: -
        OUTPUT: list of all lights objects, list of all nodes with lights
        '''
        lights = []
        light_nodes = []
        for nodes, lanes in self.edges.items():
            start_road_node_dict = self.G.nodes[nodes[0]]
            if "highway" in start_road_node_dict and start_road_node_dict["highway"] == "traffic_signals":
                for lane, path in lanes.items():
                    if path:
                        new_light_direction = self.lights_direction[str(nodes[0])] if str(nodes[0]) in self.lights_direction.keys() else None
                        new_light = Light(path.cells[0], path, new_light_direction)
                        if new_light not in path.cells[0].lights:
                            path.cells[0].addLights(new_light)
                            lights.append(new_light)
                            light_nodes.append(nodes[0])
        return list(set(lights)), list(set(light_nodes))
        # END addLights()

    def findInputOutputNodes(self):
        '''
        DESCRIPTION:
        returns two list of nodes - inputs and outputs of the map
        INPUT: -
        OUTPUT: list of input nodes
        '''
        # one direction roads
        input_nodes_one_direction = [n for n in self.G.nodes() if len([s for s in self.G.predecessors(n)]) == 0]
        output_nodes_one_direction = [n for n in self.G.nodes() if len([s for s in self.G.successors(n)]) == 0]
        
        # two direction roads
        input_output_nodes_two_direction = [n for n in self.G.nodes() if len(set([s for s in self.G.neighbors(n)])) == 1 and set([s for s in self.G.predecessors(n)])==set([s for s in self.G.successors(n)])]
        
        return list(set(input_nodes_one_direction+input_output_nodes_two_direction)), list(set(output_nodes_one_direction+input_output_nodes_two_direction))
        # END findInputOutputNodes()

    def addSelfDefinedInputNodes(self, nodes):
        '''
        DESCRIPTION: adds self-defined nodes to inputs list
        INPUT: list of self-defined input nodes
        OUTPUT: -
        '''
        self.inputs.extend(nodes)
        # END addSelfDefinedInputNodes()
        
    def addSelfDefinedOutputNodes(self, nodes):
        '''
        DESCRIPTION: adds self-defined nodes to outputs list
        INPUT: list of self-defined output nodes
        OUTPUT: -
        '''
        self.outputs.extend(nodes)
        # END addSelfDefinedOutputNodes()
        
    def assignInputProbabilities(self):
        '''
        DESCRIPTION:
        return list of probabilities corresponding to input nodes list
        INPUT: -
        OUTPUT: dict with input probabilities - node:{hour (as number id):probability}
        '''
        filename = "input_data/inputs_probabilities.json"
        inputs_probabilities = readJsonFile(filename)
        inputs_probabilities_final = inputs_probabilities["inputs"]
        for idx, i in enumerate(self.inputs):
            if str(i) not in inputs_probabilities_final.keys():
                print("Node ", i, " deleted from self.inputs -> not in inputs_probabilities_final (assignInputProbabilities())")
                del self.inputs[idx]
        return inputs_probabilities_final
        # END assignInputProbabilities()
        
    def assignOutputProbabilities(self):
        '''
        DESCRIPTION:
        return list of probabilities corresponding to output nodes list
        INPUT: -
        OUTPUT: list with output probabilities (corresponding to list self.outputs)
        '''
        filename = "input_data/outputs_probabilities.json"
        outputs_probabilities = readJsonFile(filename)
        outputs_probabilities_final = outputs_probabilities["outputs"]
        outputs_probabilities_list = []
        outputs_tmp = self.outputs.copy()
        for o in self.outputs:
            if str(o) not in outputs_probabilities_final.keys():
                print("Node ", o, " deleted from self.outputs -> not in outputs_probabilities_final (assignOutputProbabilities())")
                del outputs_tmp[outputs_tmp.index(o)]
            else:
                outputs_probabilities_list.append(outputs_probabilities_final[str(o)])
        self.outputs = outputs_tmp
        return outputs_probabilities_list
        # END assignOutputProbabilities()
        
    def showRoadNetwork(self, cells=False, inputs=False, outputs=False, lights=False, lanes=False, segments=False):
        '''
        DESCRIPTION: method to show road network
        INPUT: inputs=False, outputs=False, lights=False - determine if the following elements are visible on the map (default is that not)
        OUTPUT: road network visualization
        '''
        fig, ax = ox.plot_graph(self.G, fig_height=20, node_color='black', node_size = 10, show=False, close=False)
        cells = True if lanes or cells else False
        for edge_nodes, edge_paths in self.edges.items():
            if cells:
                edge_path_1 = []
                edge_path_2 = []
                edge_path_3 = []
                if edge_paths["third"]:
                    edge_path_3 = edge_paths["third"]
                    edge_path_2 = edge_paths["second"]
                    edge_path_1 = edge_paths["first"]
                elif edge_paths["second"]:
                    edge_path_2 = edge_paths["second"]
                    edge_path_1 = edge_paths["first"]
                elif edge_paths["first"]:
                    edge_path_1 = edge_paths["first"]
                else:
                    print("Wrong edge_path, edge_nodes:")
                    print(edge_nodes)
                if edge_path_1:
                    edge_path_lon_1 = [c.getCoords()[0] for c in edge_path_1.cells]
                    edge_path_lat_1 = [c.getCoords()[1] for c in edge_path_1.cells]
                    ax.scatter(edge_path_lon_1, edge_path_lat_1, c='red', marker = 'o', s=5)
                if edge_path_2 and lanes:
                    edge_path_lon_2 = [c.getCoords()[0] for c in edge_path_2.cells]
                    edge_path_lat_2 = [c.getCoords()[1] for c in edge_path_2.cells]
                    ax.scatter(edge_path_lon_2, edge_path_lat_2, c='green', marker = 'o', s=5)
                if edge_path_3 and lanes:
                    edge_path_lon_3 = [c.getCoords()[0] for c in edge_path_3.cells]
                    edge_path_lat_3 = [c.getCoords()[1] for c in edge_path_3.cells]
                    ax.scatter(edge_path_lon_3, edge_path_lat_3, c='blue', marker = 'o', s=5)
            else:
                break
        if inputs:
            for i in self.inputs:
                ax.scatter(self.G.nodes[i]["x"], self.G.nodes[i]["y"], c='yellow', marker = 'o', s=50)
        if outputs:
            for o in self.outputs:
                ax.scatter(self.G.nodes[o]["x"], self.G.nodes[o]["y"], c='violet', marker = 'o', s=50)
        if lights:
            for l in self.light_nodes:
                ax.scatter(self.G.nodes[l]["x"], self.G.nodes[l]["y"], c='red', marker = 'o', s=50)
        if segments:
            for vl in self.lon_segment_borders:
                ax.vlines(vl, self.min_lat, self.max_lat)
            for hl in self.lat_segment_borders:
                ax.hlines(hl, self.min_lon, self.max_lon)
        plt.axis([self.min_lon, self.max_lon, self.min_lat, self.max_lat])
        plt.show()
        #END method showRoadNetwork()
        
    def generatePaths(self):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: paths, paths_probabilities, path_keys
        '''
        paths = {}
        path_keys = {}
        for i in self.inputs:
            for o in self.outputs:
                try:
                    path_nodes = nx.shortest_path(self.G, i, o)
                except:
                    print(i)
                    print(o)
                    print("except")
                    continue
                start_node = path_nodes[0]
                #print(i, " ", o)
                paths[(i, o)] = {k:Path([]) for k in self.lane_keys}
                if i in path_keys.keys():
                    path_keys[i].append(o)
                else:
                    path_keys[i] = [o]
                for n in path_nodes[1:]:
                    paths[(i, o)]["first"].cells.extend(self.edges[start_node, n]["first"].cells)
                    '''
                    for l in self.lane_keys:
                        if self.edges[(start_node, n)][l]:
                            paths[(i, o)][l].cells.extend(self.edges[start_node, n][l].cells)
                        else:
                            first_lane_len = len(paths[(i, o)][self.lane_keys[0]].cells)
                            paths[(i, o)][l].cells.extend([None]*first_lane_len)
                    '''
                    start_node = n
        return paths, path_keys
        #END method generatePaths()
        
    def calculateSegmentBorders(self, mini, maxi, step):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: 
        '''
        segment_borders = [mini]
        next_step = segment_borders[-1]+step
        while next_step < maxi:
            segment_borders.append(next_step)
            next_step = segment_borders[-1]+step
        segment_borders.append(maxi)
        return segment_borders
        #END method calculateSegmentBorders()

    def generatePollutionSegments(self):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: 
        '''
        lon_distance = self.calculateDistance(self.min_lon, self.min_lat, self.max_lon, self.min_lat)
        lat_distance = self.calculateDistance(self.min_lon, self.min_lat, self.min_lon, self.max_lat)
        self.lon_segments_no = lon_distance / self.segment_size
        self.lat_segments_no = lat_distance / self.segment_size
        
        lon_difference = self.max_lon - self.min_lon
        lat_difference = self.max_lat - self.min_lat
        
        lon_step = lon_difference/self.lon_segments_no
        lat_step = lat_difference/self.lat_segments_no
        lon_segment_borders = self.calculateSegmentBorders(self.min_lon, self.max_lon, lon_step)
        lat_segment_borders = self.calculateSegmentBorders(self.min_lat, self.max_lat, lat_step)
        
        t = time.time()
        segments = [ [Segment(lon_segment_borders[x], lon_segment_borders[x+1], lat_segment_borders[y], lat_segment_borders[y+1]) for y,lat in enumerate(lat_segment_borders[:-1])] \
                        for x,lon in enumerate(lon_segment_borders[:-1]) ]
        #print(time.time()-t)
        return lon_segment_borders, lat_segment_borders, segments
        #END method generatePollutionSegments()
        
    def findCellSegment(self, cell_lon, cell_lat):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: 
        '''
        lon_difference = self.max_lon - self.min_lon
        lat_difference = self.max_lat - self.min_lat
        lon_segment_idx = int(((cell_lon-self.min_lon)/lon_difference)*self.lon_segments_no)
        lat_segment_idx = int(((cell_lat-self.min_lat)/lat_difference)*self.lat_segments_no)
        return self.segments[lon_segment_idx][lat_segment_idx]
        #END method findCellSegment()
        
    def assignSegmentToCells(self):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: 
        '''
        for nodes, lanes in self.edges.items():
            for lane, path in lanes.items():
                if path:
                    for c in path.cells:
                        cell_lon, cell_lat = c.getCoords()
                        cell_segment = self.findCellSegment(cell_lon, cell_lat)
                        c.segment = cell_segment
                        #cell_segment.addCell(c)
        #END method assignSegmentToCells()
        
    def findPollutantMaxSegments(self, pollutant, segments_no):
        '''
        DESCRIPTION: returns a segment with max value of pollution for provided pollutant
        INPUT: pollutant
        OUTPUT: segment with max value of pollution for provided pollutant
        '''
        segments = self.segments.copy()
        max_segments = []
        if pollutant not in segments[0][0].summary_pollution.keys():
            print("Wrong pollutant in new_grid.py/findPollutantMaxAverageSegment()")
            return None
        else:
            for i in range(segments_no):
                max_segment = segments[0][0]
                max_segment_pollution = sum(segments[0][0].summary_pollution_history[pollutant])
                for seg in segments:
                    for s in seg:
                        current_segment_pollution = sum(s.summary_pollution_history[pollutant])
                        if current_segment_pollution > max_segment_pollution:
                            max_segment = s
                segment_dict = {"history":[i for i in max_segment.summary_pollution_history[pollutant]],
                                "start_lon":max_segment.start_lon,
                                "end_lon":max_segment.end_lon,
                                "start_lat":max_segment.start_lat,
                                "end_lat":max_segment.end_lat,
                                }
                max_segments.append(segment_dict)
                for k in max_segment.summary_pollution_history.keys():
                    max_segment.summary_pollution_history[k] = []
        return max_segments
        #END method findPollutantMaxSegments()
#END class RoadNetwork()

#__________________________________________________________________________________________________
class Segment():
    _counter = -1
    def __init__(self, start_lon, end_lon, start_lat, end_lat):
        Segment._counter += 1
        self.id = Segment._counter
        self.start_lon = start_lon
        self.end_lon = end_lon
        self.start_lat = start_lat
        self.end_lat = end_lat
        self.summary_pollution = {"NO":0, "PM":0, "CO":0, "HC":0} #jednostka to gram
        self.cells = []
        self.summary_pollution_history = {"NO":[], "PM":[], "CO":[], "HC":[]} #jednostka to gram

    def addCell(self, cell):
        '''
        DESCRIPTION: add new cell to the list of cells within segment, if not already present in the list of cells
        INPUT: new cell
        OUTPUT: -
        '''
        if cell not in self.cells:
            self.cells.append(cell)
    #END method addCell()
#END class Segment()