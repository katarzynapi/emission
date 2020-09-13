from enum import Enum

class CellType(Enum):
    NULL = 0
    PAVEMENT = 1
    GRASS = 2
    BUILDING = 3
    ROAD = 4
    TRACKS = 5
    BIKE_LANE = 6
    
class Colour(Enum):
    RED = 1
    GREEN = 2

class IfBorder(Enum):
    NOT = 0
    INPUT = 1
    OUTPUT = 2

class PathType(Enum):
    MAIN = 1
    ALTERNATIVE = 2
    OPPOSITE = 3
    
class LightType(Enum):
    NULL = 0
    NORTH = 1
    EAST = 2
    SOUTH = 3
    WEST = 4