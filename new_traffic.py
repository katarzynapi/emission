import numpy as np
#For operations with longitude and latidute
from math import sin, cos, sqrt, atan2, radians, acos, floor, ceil
#Matplotlib for visualisation and animation
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#OSMNX for downloading data from OSM and creation of maps. 
import osmnx as ox
import networkx as nx
import random
import enums
import time
import datetime
import new_grid as ng

class Vehicle():
    _counter = -1
    far_away = 10000
    def __init__(self, no, pm, co, hc):
        Vehicle._counter += 1
        self.id = Vehicle._counter
        self.simulation_factor = 1
        self.location = []              # lista Cell; 1szy el. - head
        self.max_velocity = self.set_max_velocity()
        self.velocity = self.max_velocity
        self.acceleration = 0
        self.max_acceleration = 2*self.simulation_factor       # zakładam, że auto może przyspieszać 4 m/s^2
        self.max_deceleration = -1*self.simulation_factor
        self.length = 5
        self.visibility = 0           # z jakiej odległości widać agenta
        self.field_of_view = 80        # na jaką odległość widzi agent
        self.destination = None
        self.all_paths = {}
        self.path = None                  # pierwsza komórka w path to któryś z przednich sąsiadów head
        self.in_out = None
        self.lights = []    # światła dodawane w metodzie generateAgent() w obiektach typu TrafficModel()
        self.how_many_cells_forward = 0
        self.new_velocity = 0
        self.new_acceleration = 0
        self.color = None
        self.type = None
        self.emission = {"NO":no, "PM":pm, "CO":co, "HC":hc} # jednostka g/m, w raporcie jest g/km->przeliczyć, jak będę stosować inne odległości miedzy komórkami to przeliczyć
    
    def set_max_velocity(self, max_velocity="50"):
        return int(ceil(int(max_velocity)*10/36))*self.simulation_factor
        
    def init_location_empty(self):
        agents_on_init_location = [c.agent for c in self.path.cells[:self.length]]
        for a in agents_on_init_location:
            if a:
                return False
        return True

    def place_on_grid(self):
        '''self.in_out = which_path
        self.path = self.all_paths[which_path]
        self.location.insert(0,tail_position)
        tail_position.agent = self
        cell_to_add = tail_position
        for i in range(self.how_many_cells()-1):
            cell_to_add = cell_to_add.f_neighbour
            cell_to_add.agent = self
            self.location.insert(0,cell_to_add)
        path_to_remove = self.path.cells[:self.how_many_cells()+1]
        for c in path_to_remove:
            c.agent = None
        '''
        place = reversed(self.path.cells[0:self.length])
        self.location.extend(place)
        self.max_velocity = self.set_max_velocity(self.location[0].maxspeed)
        # remove from path the fragment on which the car is standing
        for c in self.path.cells[:self.length]:
            c.agent = self
        agents_on_init_location = [(c, " ", c.agent) for c in self.path.cells[:self.length]]
        #print("agents_on_init_location")
        #print(agents_on_init_location)
        self.path.cells = self.path.cells[self.length:]

    def ascribe_paths(self, paths_dict):
        #self.all_paths = paths_list
        #self.path = next((p for p in paths_list if p.type == PathType.MAIN), None)
        for in_out, p in paths_dict.items():
            self.all_paths[in_out] = Path()
            self.all_paths[in_out].type = p.type
            self.all_paths[in_out].cells = []
            for c in p.cells:
                self.all_paths[in_out].cells.append(c)
        #self.path = next((p for p in self.all_paths if p.type == enums.PathType.MAIN), None)

    @property
    def head(self):
        if not self.location:
            return None
        return self.location[0]

    @head.setter
    def head(self, cell):
        self.location.insert(0,Cell)

    @property
    def tail(self):
        if not self.location:
            return None
        return self.location[-1]

    @tail.setter
    def tail(self, cell):
        self.location.append(cell)

    def how_many_cells(self):
        return round(self.length)

    def stopping_dist(self, dec):
        # -abs(dec) jest po to, żeby opóźnienie można było podawać z minusem lub bez
        return sum(   list(  range( self.velocity, 0, -abs(dec) )  )   )

    def reached_destination(self):
        #jeśli przód agenta jest blisko celu, to cel uznajemy za osiągnięty
        #print("self.location dla agenta id = ", self.id)
        #print(self.location)
        if not self.location:
            return True
        if self.location[0]:
            x1, y1 = self.location[0].getCoords()
            x2, y2 = self.destination.getCoords()
            if ox.euclidean_dist_vec(y1, x1, y2, x2)*100000 < self.length:
                #print("distance_to_destination")
                #print(ox.euclidean_dist_vec(y1, x1, y2, x2)*100000)
                #if self.head.f_neighbour == None:
                return True
            else:
                return False
        else:
            return False

    def alternative_route_exists(self):
        for p in self.all_paths:
            if (p.type == enums.PathType.ALTERNATIVE and (self.head.r_neighbour in p.cells or self.head.r_neighbour in p.cells)):
                return True
        return False

    def opposite_route_exists(self):
        for p in self.all_paths:
            if p.type == enums.PathType.OPPOSITE and (self.head.r_neighbour in p.cells or self.head.r_neighbour in p.cells):
                return True
        return False

    def can_safely_change_to_other_path(self, other_path_fwd, other_path_bwd):
        #gdyby się okazało, że to wszystko wali błędami, to przerobimy na zwykłą pętlę for
        (prev_dist, prev_vehicle) = next(((len(other_path_bwd)-1-i, x.agent) for i, x in enumerate(other_path_bwd[::-1]) if x.agent is not None), (None, None))
        (next_dist, next_vehicle) = next(((i+1, x.agent) for i, x in enumerate(other_path_fwd) if x.agent is not None), (None, None))
        if prev_vehicle is None:
            prev_safe = True
        else:
            prev_safe = (prev_vehicle.stopping_dist(-2) + prev_dist - self.how_many_cells() > self.stopping_dist(-2))
        if next_vehicle is None:
            next_safe = True
        else:
            next_safe = (self.stopping_dist(-2) + next_dist > next_vehicle.stopping_dist(-2))
        return (next_safe and prev_safe)

    def visible_path(self):
        # powinno zwracać ten fragment wyznaczonej trasy, który agent widzi
        return self.path.cells
        #return [c for c in self.path if self.location[0].coords.calculate_distance(c.coords) > self.field_of_view]

    def scan_for_obstacles(self):
        current_obstacles = []
        for c in self.visible_path():
            for o in c.lights:
                if self.in_out in o.connected_paths:
                    current_obstacles.append(o)
        self.obstacles = current_obstacles

    def calc_path_distance(self, obstacle):
        #print("calc_path_distance(self, obstacle)")
        for i, c in enumerate(self.visible_path()):
            if obstacle in c.lights or obstacle == c.agent:
                return i+1
        return 0

    def find_first_agent_on_path(self):
        return next((x.agent for x in self.visible_path() if x.agent is not None), None)

    def find_nearest_lights(self):
        return next((obs for obs in self.lights), None)

    def find_nearest_blockade(self):
        return next((obs for obs in self.obstacles if isinstance(obs, Blockade)), None)

    def update_route(self):
        if self.path.type == enums.PathType.OPPOSITE:
            neighbour_main = self.head.neighbour_r
            main_path = next((p for p in self.all_paths if p.type == enums.PathType.MAIN), None)
            start = main_path.index(neighbour_main)
            main_fwd = main_path[start+1:]
            main_bwd = main_path[:start+1]
            if can_safely_change_to_other_path(main_fwd, main_bwd):
                self.path.type = enums.PathType.MAIN
                self.path.cells = main_bwd
        else:
            if self.path.type == enums.PathType.ALTERNATIVE and self.path.cells[-2].if_border == enums.IfBorder.NOT and len(self.path.cells)<self.stopping_dist(-1)+10:
                neighbour_main = self.head.r_neighbour
                for k, p in self.all_paths.items():
                    #print(k, p.type)
                    pass
                main_path = next((p for k, p in self.all_paths.items() if p.type == enums.PathType.MAIN), None)
                #print("main_path: ", main_path)
                start = main_path.cells.index(neighbour_main)
                main_fwd = main_path.cells[start+1:]
                main_bwd = main_path.cells[:start+1]
                if self.can_safely_change_to_other_path(main_fwd, main_bwd):
                    self.path = main_path
                    self.path.type = enums.PathType.MAIN
                    self.path.cells = main_fwd
            #if self.path.type == enums.PathType.ALTERNATIVE and self.tail.r_neighbour:
                #if not self.tail.r_neighbour.obstacles:
            else:
                blockade = self.find_nearest_blockade()
                if blockade is not None and self.calc_path_distance(blockade) < 3:
                    if self.path.type == enums.PathType.ALTERNATIVE:
                        neighbour_main = self.head.r_neighbour if self.head.r_neighbour is not None else self.head.l_neighbour
                        main_path = next((p for k, p in self.all_paths.items() if p.type == enums.PathType.MAIN), None)
                        start = main_path.cells.index(neighbour_main)
                        main_fwd = main_path.cells[start+1:]
                        main_bwd = main_path.cells[:start+1]
                        if self.can_safely_change_to_other_path(main_fwd, main_bwd):
                            self.path = main_path
                            self.path.type = enums.PathType.MAIN
                            self.path.cells = main_fwd
                    else:
                        avail_alt = [p for k, p in self.all_paths.items() if p.type == enums.PathType.ALTERNATIVE and (self.head.r_neighbour in p.cells or self.head.l_neighbour in p.cells)]
                        for p in avail_alt: # zmiana avail_path na avail_alt
                            neighbour_alt = self.head.r_neighbour if self.head.r_neighbour in p.cells else self.head.l_neighbour #zmiana p na p.cells
                            start = p.cells.index(neighbour_alt) #zmiana p na p.cells
                            alt_fwd = p.cells[start+1:] #zmiana p na p.cells
                            alt_bwd = p.cells[:start+1] #zmiana p na p.cells
                            if self.can_safely_change_to_other_path(alt_fwd, alt_bwd):
                                self.path = p
                                self.path.type = enums.PathType.ALTERNATIVE
                                self.path.cells = alt_fwd
        self.scan_for_obstacles()


    def compute_new_location(self):
        self.how_many_cells_forward = int(round(self.new_velocity))

    def compute_new_velocity(self):
        #print("W compute_new_velocity weszło do:")
        #print("new_acceleration:")
        #print(self.new_acceleration)
        if self.dist <= 1 and self.new_acceleration == 0 and (self.velocity == 1 or self.velocity == 0):
            self.new_velocity = 0
        else:
            new_velocity = self.velocity + self.new_acceleration
            self.new_velocity = new_velocity if new_velocity>=0 else 0
        #if (self.new_acceleration >= 0):
            #print("if")
            #print("self.velocity:")
            #print(self.velocity)
            #print("self.self.acceleration:")
            #print(self.acceleration)
            #print("self.max_velocity:")
            #print(self.max_velocity)
            #self.new_velocity = min(self.velocity + self.new_acceleration, self.max_velocity)
        #else:
            #print("else")
            #self.new_velocity = max(self.velocity + self.new_acceleration, 0)
        #print("new_velocity")
        #print(self.new_velocity)

    #gdzieś w tej funkcji radośnie przesądzam i trochę hardcoduję, że opóźnienie może być tylko -1 lub -2
    def compute_new_acceleration(self):
        #print("W compute_new_acceleration weszło do:")
        agent_on_path = self.find_first_agent_on_path()
        lights = self.find_nearest_lights()
        #print("lights")
        #print(lights)
        blockade = self.find_nearest_blockade()
        
        l_dist = self.far_away           #jakaś losowa duża liczba - oznacza, że świateł/agenta nie ma (w założeniu nic nie jest tak daleko)
        a_dist = self.far_away
        b_dist = self.far_away
        
        if lights is not None and lights.colour == enums.Colour.RED:
            l_dist = self.calc_path_distance(lights)
            #print("l_dist")
            #print(l_dist)
        if agent_on_path is not None:
            a_dist = self.calc_path_distance(agent_on_path) + agent_on_path.velocity
            #print("a_dist")
            #print(a_dist)
        if blockade is not None:
            b_dist = self.calc_path_distance(blockade)
        self.dist = min(a_dist, l_dist, b_dist)
        # zatrzymanie tuż przed przeszkodą  
        if self.dist <= 1 and (self.velocity == 1 or self.velocity == 0):
            self.new_acceleration = 0
            #print("zatrzymanie tuż przed przeszkodą, dist = ", self.dist)
            #print("self.dist <= 1 and (self.velocity == 1 or self.velocity == 0)")
        # nie zdąży wyhamować
        elif self.dist > 1 and self.dist < self.stopping_dist(self.max_deceleration)-self.velocity:
            if self.velocity < self.max_velocity: #przespieszenie, żeby przejechać jeszcze szybciej
                self.new_acceleration = self.max_acceleration
            elif self.velocity == self.max_velocity: #ale nie szybciej niż prędkość maksymalna
                self.new_acceleration = 0
            elif self.velocity > self.max_velocity: #albo wręcz zwolnić, jeśli pojawiło się ograniczenie i jedziemy za szybko
                self.new_acceleration = self.max_deceleration
            else:
                print("Wrong condition in 'nie zdąży wyhamować' in new_traffic.py/compute_new_acceleration(), dist = ", self.dist)
            #print("nie zdąży wyhamować, dist = ", self.dist)
            #old version: self.new_acceleration = self.max_acceleration if self.velocity < self.max_velocity else 0
            #print("self.dist > 1 and self.dist < self.stopping_dist(-1)-self.velocity")
        # zdążymy wyhamować przed przeszkodą i zwalnianiamy
        elif self.dist >= self.stopping_dist(self.max_deceleration)-self.velocity and self.dist <= self.stopping_dist(self.max_deceleration):
            self.new_acceleration = self.max_deceleration
            #print("zdążymy wyhamować przed przeszkodą i zwalnianiamy, dist = ", self.dist)
            #print("self.dist >= self.stopping_dist(-1)-self.velocity and self.dist <= self.stopping_dist(-1)")
        # utrzymywanie stałej prędkości jak zaczynamy "widzieć przeszkodę"
        elif self.dist > self.stopping_dist(self.max_deceleration) and self.dist <= self.stopping_dist(self.max_deceleration)*2+1:
            if self.velocity <= self.max_velocity: #utrzymanie prędkości, jeśli jest zgodna z przepisami
                self.new_acceleration = 0
            elif self.velocity > self.max_velocity: #zwolnienie, gdy pojawiło się ograniczenie prędkości
                self.new_acceleration = self.max_deceleration
            else:
                print("Wrong condition in 'utrzymywanie stałej prędkości jak zaczynamy widzieć przeszkodę' in new_traffic.py/compute_new_acceleration()")
            #print("utrzymywanie stałej prędkości jak zaczynamy widzieć przeszkodę, dist = ", self.dist)
            #old version: self.new_acceleration = 0
            #print("self.dist > self.stopping_dist(-1) and self.dist <= self.stopping_dist(-1)*2")
        # dążenie do max_velocity, gdy przeszkoda jest daleko
        elif self.dist > self.stopping_dist(self.max_deceleration)*2+1:
            if self.velocity < self.max_velocity: #przespieszanie do uzyskania prędkości maksymalnej
                self.new_acceleration = self.max_acceleration
            elif self.velocity == self.max_velocity: #utrzymanie prędkości maksymalnej
                self.new_acceleration = 0
            elif self.velocity > self.max_velocity: #zwalnianie, jeśli pojawiło się ograniczenie i jedziemy za szybko
                self.new_acceleration = self.max_deceleration
            else:
                print("Wrong condition in 'dążenie do max_velocity, gdy przeszkoda jest daleko' in new_traffic.py/compute_new_acceleration()")
            #print("dążenie do max_velocity, gdy przeszkoda jest daleko, dist = ", self.dist)
            #old version: self.new_acceleration = self.max_acceleration if self.velocity < self.max_velocity else 0
            #print("dist > self.stopping_dist(-1)*2")
        else:
            self.new_acceleration = max(0, self.new_acceleration)
            '''
            if self.velocity < self.max_velocity: #przespieszanie do uzyskania prędkości maksymalnej
                self.new_acceleration = self.max_acceleration
            elif self.velocity == self.max_velocity: #utrzymanie prędkości maksymalnej
                self.new_acceleration = 0
            elif self.velocity > self.max_velocity: #zwalnianie, jeśli pojawiło się ograniczenie i jedziemy za szybko
                self.new_acceleration = self.max_deceleration
            else:
                print("Wrong condition in 'else' in new_traffic.py/compute_new_acceleration()")
            '''
            #print("else, self.new_acceleration: ", self.new_acceleration)
            #print("self.dist: ", self.dist)
        #print("self.new_acceleration:")
        #print(self.new_acceleration)
    
    def addPollutionToSegments(self, polluted_cells):
        for c in polluted_cells:
            for s in c.pollution.keys():
                #c.pollution[s] += self.emission[s]/1000
                c.segment.summary_pollution[s] += self.emission[s]/1000
    
    def update_position(self):
        #print("Funkcja update_position")
        # powinno działać. testy by nie zaszkodziły
        # auto wjeżdża na tyle komórek path, ile wyznaczono na podstawie prędkości
        if not self.location[0]:
            #print("Agent ", self.id, " is still pending to enter map")
            return False
        #print("self.how_many_cells_forward")
        #print(self.how_many_cells_forward)
        vel_len_diff = self.how_many_cells_forward - self.length
        #print("vel_len_diff")
        #print(vel_len_diff)
        start = vel_len_diff if vel_len_diff >= 0 else 0
        #print("start")
        #print(start)
        new_car_part = self.path.cells[start:self.how_many_cells_forward]
        #print("new_car_part")
        #print(new_car_part)
        new_car_part.reverse()
        #print("new_car_part.reverse()")
        #print(new_car_part)
        if not new_car_part and self.dist <= 1:
            print("location (x, y): ", self.location[0].getCoords())
            print("Wchodzę do pass")
            pass
        else:
            # dodajemy agenta dla nowych komórek
            #print("Wchodzę do else")
            for cell in self.path.cells[start:self.how_many_cells_forward]:
                cell.agent = self
            # auto pozostaje na swoich komórkach - tyle, o ile się przesuwa
            old_car_part = self.location[:-self.how_many_cells_forward]
            # zdejmujemy agenta z komórek, z których zjechał
            for cell in self.location[-self.how_many_cells_forward:]:
                cell.agent = None
            # zebranie komórek, nad którymi przejechał samochód w danej iteracji
            travelled_cells = self.location + self.path.cells[:self.how_many_cells_forward]
            nearest_lights = self.find_nearest_lights()
            if nearest_lights in [c.lights[0] for c in travelled_cells if c.lights]:
                #print("nearest_lights: ", nearest_lights)
                del self.lights[self.lights.index(nearest_lights)]
            # nowe położenie
            self.location = new_car_part + old_car_part
            if self.location: # bo wywalało czasami na IndexError: list index out of range, jak pusto, to max_velocity bez zmian
                self.max_velocity = self.set_max_velocity(self.location[0].maxspeed)
            # dodanie zanieczyszczeń do komórek, nad którymi przejechał samochód w danej iteracji
            self.addPollutionToSegments(travelled_cells)
            #print("old_car_part")
            #print(old_car_part)
            #print("new_car_part")
            #print(new_car_part)
            #print("self.location")
            #print(self.location)
            # ucinamy z path to, na co wjechało auto
            self.path.cells = self.path.cells[self.how_many_cells_forward:]
            #print("self.path.cells")
            #print(self.path.cells[:30])
        #print("Koniec funkcji update_position")


    #aby wszyscy ruszali się jednocześnie, najpierw trzeba policzyć nowe parametry, a jak wszystkie będą policzone, to je przypisać. Dlatego dwie metody (wzorowałam się na SimultaneousActivation z mesa)

    def step(self):
        self.scan_for_obstacles()
        self.update_route()
        self.compute_new_acceleration()
        self.compute_new_velocity()
        self.compute_new_location()

    def advance(self):
        self.update_position()
        self.velocity = self.new_velocity
        self.acceleration = self.new_acceleration

#END class generalAgent()

#__________________________________________________________________________________________________

class TrafficModel:
    def __init__(self, road_network, simulation_duration, start_time, interval):
        self.image = None
        self.steps = 0
        self.time = 0
        self.agents = []
        self.road_network = road_network
        self.fig, self.ax = ox.plot_graph(self.road_network.G, fig_height=20, node_color='black', node_size = 10, show=False, close=False)
        self.hlines = [self.ax.hlines(hl, self.road_network.min_lon, self.road_network.max_lon, linewidth=0.2) for hl in self.road_network.lat_segment_borders]
        self.vlines = [self.ax.vlines(vl, self.road_network.min_lat, self.road_network.max_lat, linewidth=0.2) for vl in self.road_network.lon_segment_borders]
        self.emission_value = [[self.ax.text((s.start_lon+s.end_lon)/4, s.start_lat, s.summary_pollution["CO"]) for s in seg] for seg in self.road_network.segments]
        self.segment_colours = [[self.ax.fill_between([s.start_lon,s.end_lon], [s.start_lat]*2, [s.end_lat]*2, color='#ffffff') for s in seg] for seg in self.road_network.segments]
        self.lines = [self.ax.plot([19.92702, 19.92701], [50.06670, 50.06669], linewidth=1)[0]]
        #self.paths = self.road_network.paths
        self.lights = road_network.lights
        self.points = [self.ax.scatter(c.location.getCoords()[0], c.location.getCoords()[1], c='red', marker = 'o', s=10) for c in self.lights]
        self.obstacles = []
        self.destination_reached = False
        self.vehicles_distribution = self.getVehiclesDistribution()
        self.emission_param = self.getEmissionParam()
        self.i = 0 #global iteration
        self.start_time = start_time
        self.quater_iter = self.transformHourToQuaterIter(self.start_time) #quater iteration - first is 00:00-00:15 etc (max is 96)
        self.pending_agents = []
        self.step_duration = 1 # in seconds
        self.simulation_duration = simulation_duration
        self.k = 15*self.step_duration*60
        self.date_time = datetime.datetime.now().strftime("%d%m%Y_%H%M%S")
        self.interval = interval
        self.initializeEmissionFile()
    
    def initializeEmissionFile(self):
        with open("emission_data/emission_"+self.date_time+".txt",'ab') as f:
            np.savetxt(f, ["start_time: "+self.start_time], fmt="%s")
            np.savetxt(f, ["interval_duration: "+str(self.interval)], fmt="%s")
            np.savetxt(f, ["simulation_duration: "+str(self.simulation_duration)], fmt="%s")
            np.savetxt(f, ["segment_size: "+str(self.road_network.segment_size)], fmt="%s")
    
    def transformHourToQuaterIter(self, time):
        '''
        DESCRIPTION: method to tranform time in form "hh:mm" to quater_iter
        INPUT: time in form "hh:mm"
        OUTPUT: quater_iter
        '''
        time_h, time_m = time.split(":")
        time_h = int(time_h)
        time_h = time_h if time_h<=23 and time_h>=0 else 0
        time_m = int(time_m)
        time_m = time_m if time_m<=59 and time_h>=0 else 0
        return time_h*4+1 + int(time_m/15)
        # END method transformHourToQuaterIter()

    def saveSimulationSnapshot(self):
        '''
        DESCRIPTION: method to show current state of simulation on the map
        INPUT: 
        OUTPUT: map with current state of simulation
        
        for p,l in zip(self.points, self.lights):
            colour = 'red' if l.colour == enums.Colour.RED else 'green'
            p.set_color(colour)
        '''
        current_segment_colours = self.getSegmentColours()
        for seg,e_val,s_col,curr_s_col in zip(self.road_network.segments, self.emission_value, self.segment_colours, current_segment_colours):
            for s,e_v,s_c,c_s_c in zip(seg,e_val,s_col,curr_s_col):
                e_v.set_text(round(s.summary_pollution["CO"], 4))
                s_c.set_color(c_s_c)
        coords = self.calculateAgentsCoordsForVisualization()
        for i, a in enumerate(coords):
            if i < len(self.lines):
                self.lines[i].set_xdata(a[0])
                self.lines[i].set_ydata(a[1])
            else:
                self.lines.append(self.ax.plot(a[0], a[1], linewidth=4)[0])
        emission_array = np.asarray([[s.summary_pollution["CO"] for s in seg[::-1]] for seg in self.road_network.segments]).transpose()
        with open("emission_data/emission_"+self.date_time+".txt",'ab') as f:
            np.savetxt(f, emission_array, fmt="%4.4f", delimiter=",", header=str(self.i))
        for seg in self.road_network.segments:
            for s in seg:
                for pol in s.summary_pollution.keys():
                    s.summary_pollution_history[pol].append(s.summary_pollution[pol])
                    s.summary_pollution[pol] = 0.0
        image_name = 'simulation_snapshots/simulation_state_'+self.date_time+"_"+str(self.i)+'.jpg'
        plt.savefig(image_name, dpi=300)
        # END method showSimulationSnapshot()
        
    def animation(self, coords):
        # lista linii, może za każdym razem tworzyć od nowa z użycie plot
        #self.lines = []
        print("____________________________________")
        print("Animation i = ", self.i)
        print("self.quater_iter: ", self.quater_iter)
        print("Agents number: ", len(self.agents))
        print("____________________________________")
        #print("self.lines PRZED for loop: ", self.lines, " odpowiadają kolejno agentom ", self.agents)
        if self.i % self.interval == 0 or self.i == self.simulation_duration-1:
            emission_array = np.asarray([[s.summary_pollution["CO"] for s in seg[::-1]] for seg in self.road_network.segments]).transpose()
            with open("emission_data/emission_"+self.date_time+".txt",'ab') as f:
                np.savetxt(f, emission_array, fmt="%4.4f", delimiter=",", header=str(self.i))
            for seg in self.road_network.segments:
                for s in seg:
                    for pol in s.summary_pollution.keys():
                        s.summary_pollution_history[pol].append(s.summary_pollution[pol])
                        s.summary_pollution[pol] = 0.0
        for p,l in zip(self.points, self.lights):
            colour = 'red' if l.colour == enums.Colour.RED else 'green'
            p.set_color(colour)
        current_segment_colours = self.getSegmentColours()
        for seg,e_val,s_col,curr_s_col in zip(self.road_network.segments, self.emission_value, self.segment_colours, current_segment_colours):
            for s,e_v,s_c,c_s_c in zip(seg,e_val,s_col,curr_s_col):
                e_v.set_text(round(s.summary_pollution["CO"], 4))
                s_c.set_color(c_s_c)
        for i, a in enumerate(coords):
            if i < len(self.lines):
                self.lines[i].set_xdata(a[0])
                self.lines[i].set_ydata(a[1])
            else:
                self.lines.append(self.ax.plot(a[0], a[1], linewidth=4)[0])
        #print("self.lines PO for loop: ", self.lines, " odpowiadają kolejno agentom ", self.agents)
        #self.segment_colours + 
        return self.lines + self.points + self.hlines + self.vlines + self.emission_value
        #return self.lines + self.hlines + self.vlines + self.emission_value
        
    def framesOneAgent(self):
        agent_id = -1
        while not self.destination_reached:
            self.step()
            agent_location = self.agents[agent_id].location
            agent_location_lon = [c.getCoords()[0] for c in agent_location]
            agent_location_lat = [c.getCoords()[1] for c in agent_location]
            self.destination_reached = self.agents[agent_id].reached_destination()
            #print("destination_reached")
            #print(self.destination_reached)
            start_x, start_y = (19.92202, 50.06475) if self.destination_reached else (agent_location_lon[0], agent_location_lon[-1])
            end_x, end_y = (19.92701, 50.06669) if self.destination_reached else (agent_location_lat[0], agent_location_lat[-1])
            yield [([start_x, start_y], [end_x, end_y])]
    
    def showOneAgentMovement(self):
        ani = animation.FuncAnimation(
            self.fig, self.animation, frames=self.framesOneAgent, interval=500, repeat=False)
        plt.show()
        
    def framesMultipleAgents(self):
        duration = self.simulation_duration #in seconds
        time_end = time.time() + duration
        while time.time() < time_end:
            #print("\nLista agentów na początku głównej pętli while:\n", self.agents)
            #change quater iteration
            if self.i%self.k == 0 and self.quater_iter<96 and self.i != 0:
                self.quater_iter += 1
            elif self.i%self.k == 0 and self.quater_iter>=96 and self.i != 0:
                self.quater_iter = 1
            #dodaj Pending agents
            self.addPendingAgentsToModel()
            #generate agents in inputs
            self.generateStepAgents()
            self.removeReachedDestinationAgents()
            coords = self.calculateAgentsCoordsForVisualization()
            self.step()
            self.i+=1
            yield coords
    
    def showMultipleAgentsMovement(self):
        ani = animation.FuncAnimation(
            self.fig, self.animation, frames=self.framesMultipleAgents, interval=self.step_duration*1000, repeat=False)
        plt.show()
    
    def simulateMultipleAgentsMovement(self):
        '''
        DESCRIPTION: method to run the whole simulation
        INPUT: -
        OUTPUT: -
        '''
        for i in range(self.simulation_duration):
            self.i = i
            print("____________________________________")
            print("Animation i = ", self.i)
            print("self.quater_iter: ", self.quater_iter)
            print("Agents number: ", len(self.agents))
            print("____________________________________")
            if self.i % self.interval == 0 or self.i == self.simulation_duration-1:
                self.saveSimulationSnapshot()
            #change quater iteration
            if self.i%self.k == 0 and self.quater_iter<96 and self.i != 0:
                self.quater_iter += 1
            elif self.i%self.k == 0 and self.quater_iter>=96 and self.i != 0:
                self.quater_iter = 1
            #dodaj Pending agents
            self.addPendingAgentsToModel()
            #generate agents in inputs
            self.generateStepAgents()
            self.removeReachedDestinationAgents()
            self.step()
            #if self.i % 50 == 0:
                #self.saveSimulationSnapshot()
            '''
            if self.i % 10 == 0:
                emission_array = np.asarray([[s.summary_pollution["CO"] for s in seg[::-1]] for seg in self.road_network.segments]).transpose()
                with open("emission.txt",'ab') as f:
                    np.savetxt(f, emission_array, fmt="%4.4f", delimiter=",", header=str(self.i))
                for seg in self.road_network.segments:
                    for s in seg:
                        for pol in s.summary_pollution.keys():
                            s.summary_pollution[pol] = 0.0
                print("Emission array: ")
                print(emission_array)
            '''
        # END method simulateMultipleAgentsMovement()
    
    def showAgentPath(self, agent_id):
        '''
        DESCRIPTION: method to show agent path
        INPUT: agent_id - index of agent in agents_list
        OUTPUT:
        '''
        fig, ax = ox.plot_graph(self.road_network.G, fig_height=20, node_color='black', node_size = 10, show=False, close=False)
        agent_path = self.agents[agent_id].path
        agent_path_lon = [c.getCoords()[0] for c in agent_path.cells]
        agent_path_lat = [c.getCoords()[1] for c in agent_path.cells]
        ax.scatter(agent_path_lon, agent_path_lat, c='red', marker = 's', s=1)
        plt.show()
    # END method showAgentPath()
    
    def showMaxSegmentPollutionDistribution(self, pollutant, segment_no):
        '''
        DESCRIPTION:
        INPUT:
        OUTPUT:
        '''
        time_h, time_m = self.start_time.split(":")
        start_time = datetime.datetime(1, 1, 1, int(time_h), int(time_m), 0)
        interval_no = int(self.simulation_duration/self.interval)
        timeline = [str((start_time+i*datetime.timedelta(seconds=self.interval)).time()) for i,j in enumerate(range(interval_no+1))]
        plt.close()
        max_segments = self.road_network.findPollutantMaxSegments(pollutant, segment_no)
        for i, k in enumerate(max_segments):
            plt.figure(i)
            plt.plot(timeline, max_segments[i]["history"])
            plt.title(str(pollutant)+" "+str(i))
            plt.xlabel("time")
            plt.ylabel(str(pollutant)+" emission [g]")
        fig, ax = ox.plot_graph(self.road_network.G, fig_height=20, node_color='black', node_size = 10, show=False, close=False)
        colours = ['#b30000', '#e60000', '#ff1a1a', '#ff4d4d', '#ff8080', '#ffb3b3', '#ffe6e6']
        for i, s in enumerate(max_segments):
            ax.fill_between([s["start_lon"],s["end_lon"]], s["start_lat"], s["end_lat"], color=colours[i])
        plt.title(str(segment_no)+" max emission segments")
        plt.show()
    # END method showMaxSegmentPollutionDistribution()

    def getSummaryEmission(self, pollutant):
        '''
        DESCRIPTION:
        INPUT:
        OUTPUT:
        '''
        return sum([sum([sum(s.summary_pollution_history[pollutant]) for s in seg]) for seg in self.road_network.segments])
    # END method getSummaryEmission()
    
    def getSegmentColours(self):
        '''
        DESCRIPTION:
        INPUT:
        OUTPUT:
        '''
        colours_dict = {0:"#ebfaeb", 1:"#adebad", 2:"#70db70", 3:"#33cc33", 4:"#248f24", 5:"#145214"}
        max_emission = max([s.summary_pollution["CO"] for seg in self.road_network.segments for s in seg])
        if max_emission == 0:
            max_emission = 0.0001
        return [[colours_dict[floor(s.summary_pollution["CO"]/max_emission/0.2)] for s in seg] for seg in self.road_network.segments]
    # END method getSegmentColours()
    
    def getVehiclesDistribution(self):
        '''
        DESCRIPTION: reads json file with vehicles distribution and return dict with vehicle_type:probability of occurence
        INPUT: -
        OUTPUT: dict with vehicles distribution (vehicle_type:probability of occurence)
        '''
        filename = "input_data/vehicles_distribution.json"
        vehicles_distribution = ng.readJsonFile(filename)
        return vehicles_distribution["vehicles_distribution"]
    # END method getVehiclesDistribution()

    def getEmissionParam(self):
        '''
        DESCRIPTION: reads json file with vehicles emission parameters and return dict with vehicle_type:emission in g/km for no, pm, co, hc
        INPUT: -
        OUTPUT: dict with vehicles emission parameters (vehicle_type:emission in g/km for no, pm, co, hc)
        '''
        filename = "input_data/vehicles_emission.json"
        vehicles_emission = ng.readJsonFile(filename)
        return vehicles_emission["vehicles_emission_km"]
    # END method getEmissionParam()
    
    def generateStepAgents(self):
        '''
        DESCRIPTION: method to generate new agents on all inputs to the map according to probabilities
        INPUT: -
        OUTPUT: -
        '''
        for i in self.road_network.inputs:
            car_count = self.road_network.inputs_probability[str(i)][str(self.quater_iter)]
            self.generateAgent(i, car_count)
    # END method generateStepAgents()
    
    def generateAgent(self, input_node, car_count):
        '''
        DESCRIPTION: method to generate new agent and place it on the map if there is empty space on the entry path fragment or adds it to pending agents list
        INPUT:
            input_node - input node id
            car_count - number of cars to be generated in current quarter iteration
        OUTPUT: -
        '''
        car_generate_choice = [0, 1]
        prob = car_count/self.k
        weights = [1-prob, prob]
        car_generate = random.choices(car_generate_choice, weights = weights, k = 1)[0]
        #print("input_node: ", input_node)
        #print("prob: ", prob)
        if car_generate:
            no, pm, co, hc = self.generateAgentEmissionParam()
            new_agent = Vehicle(no, pm, co, hc)
            #print("Wygenerowano agenta o id = ", new_agent.id, "przy i = ", self.i)
            #print(self.road_network.path_keys)
            #print(self.road_network.paths_probability)
            output_node = random.choices(self.road_network.path_keys[input_node], weights=self.road_network.outputs_probability)[0]
            new_agent.path = ng.Path(self.road_network.paths[(input_node, output_node)]["first"].cells)
            while not new_agent.path.cells:
                #start_end_path_node = random.choice(list(self.road_network.paths.keys()))
                #print("self.road_network.paths_probability ", self.road_network.paths_probability)
                output_node = random.choices(self.road_network.path_keys[input_node], weights=self.road_network.outputs_probability)[0]
                new_agent.path = ng.Path(self.road_network.paths[(input_node, output_node)]["first"].cells)
            #print("Is generated agent pending? ", not new_agent.init_location_empty())
            for c in self.road_network.paths[(input_node, output_node)]["first"].cells:
                if c.lights:
                    new_agent.lights.extend(c.lights)
            if not new_agent.init_location_empty():
                self.pending_agents.append(new_agent)
            else:
                self.addAgentToModel(new_agent)
            #print('self.road_network.paths[start_end_path_node]["first"].cells[0]')
            #print(self.road_network.paths[start_end_path_node]["first"].cells[0])
    # END method generateAgent()
    
    def addAgentToModel(self, agent):
        '''
        DESCRIPTION: method to add new agent on the map entry
        INPUT: agent - agent object to add on the map entry
        OUTPUT: -
        '''
        self.agents.append(agent)
        agent.place_on_grid()
        agent.destination = agent.path.cells[-1]
    # END method addAgentToModel()
    
    def generateAgentEmissionParam(self):
        '''
        DESCRIPTION: 
        INPUT: 
        OUTPUT: -
        '''
        vehicle_types = list(self.vehicles_distribution.keys())
        vehicle_probabilities = [self.vehicles_distribution[v_t] for v_t in vehicle_types]
        selected_vehicle_type = random.choices(vehicle_types, weights=vehicle_probabilities)[0]
        selected_param = self.emission_param[selected_vehicle_type]
        return selected_param["NO"], selected_param["PM"], selected_param["CO"], selected_param["HC"]
    # END method generateAgentEmissionParam()
        
    def addPendingAgentsToModel(self):
        '''
        DESCRIPTION: method to add new agents on the map entry
        (used for agents from pending_agents list, which can be added, because there is empty space on the entry path fragment)
        INPUT: agent -
        OUTPUT: -
        '''
        for a in self.pending_agents:
            if a.init_location_empty():
                #a.location = []
                #print("Dodano agenta z pending o id ", a.id)
                self.addAgentToModel(a)
                self.pending_agents.remove(a)
    # END method addPendingAgentsToModel()

    def removeReachedDestinationAgents(self):
        '''
        DESCRIPTION: method to remove agents which reached their destination point, also removes lines, which correspond to those agents while visualization
        INPUT: agent -
        OUTPUT: -
        '''
        for a in self.agents:
            if not a.location or a.reached_destination():
                idx = self.agents.index(a)
                self.agents.remove(a)
                print("Destination reached")
                if idx == len(self.lines) and idx != 0:
                    del self.lines[idx-1]
                else:
                    if idx < len(self.lines):
                        self.lines[idx].set_xdata([])
                        self.lines[idx].set_ydata([])
                        del self.lines[idx]
                    else:
                        print("In removeReachedDestinationAgents() idx >= len(self.lines)")
                        print("idx = ", idx, " len(self.lines) = ", len(self.lines))
    # END method removeReachedDestinationAgents()
    
    def calculateAgentsCoordsForVisualization(self):
        '''
        DESCRIPTION: method to create list of coords for current positions of agents on the map (used for visualization)
        INPUT: -
        OUTPUT: list of coords (list of tuples, tuples are in the following form: ([lon_head, lon_tail], [lat_head, lat_tail])
        '''
        agents_locations = [a.location for a in self.agents if a.location]
        agent_locations_lon = [[c.getCoords()[0] for c in a] for a in agents_locations]
        agent_locations_lat = [[c.getCoords()[1] for c in a] for a in agents_locations]
        coords = []
        for a_lon, a_lat in zip(agent_locations_lon, agent_locations_lat):
            coords.append(([a_lon[0], a_lon[-1]], [a_lat[0], a_lat[-1]]))
        return coords
    # END method calculateAgentsCoordsForVisualization()

    def step(self):
        if self.steps%80==0:
            for l in self.lights:
                if l.type == enums.LightType.NORTH or (l.type == enums.LightType.WEST and l.colour == enums.Colour.GREEN):
                    l.changeColour()
        if self.steps%80==20:
            for l in self.lights:
                if l.type == enums.LightType.EAST or l.type == enums.LightType.NORTH:
                    l.changeColour()
        if self.steps%80==40:
            for l in self.lights:
                if l.type == enums.LightType.SOUTH or l.type == enums.LightType.EAST:
                    l.changeColour()
        if self.steps%80==60:
            for l in self.lights:
                if l.type == enums.LightType.WEST or l.type == enums.LightType.SOUTH:
                    l.changeColour()
        for a in self.agents:
            a.step()
            if a.location:
                pass
                #print("Agent id = ", a.id, "x")
                #print("Velocity:")
                #print(a.velocity*3.6)
                #print("Max velocity:")
                #print(a.max_velocity*3.6)
                #print("")
                #print("New_velocity:")
                #print(a.new_velocity)
                #print("Accel:")
                #print(a.acceleration)
                #print("New_accel:")
                #print(a.new_acceleration)
                #print("")
        for a in self.agents:
            a.advance()
        self.steps += 1
        self.time += self.step_duration
    # END method step()

#END class TrafficModel()

#__________________________________________________________________________________________________
