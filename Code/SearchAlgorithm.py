# This file contains all the required routines to make an A* search algorithm.
#
__authors__='ARNAU_ALBERT'
__group__='DJ15.04'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Grau en Enginyeria Informatica
# Curs 2016- 2017
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from math import *


class Node:
    # __init__ Constructor of Node Class.
    def __init__(self, station, father):
        """
        __init__:   Constructor of the Node class
        :param
                - station: STATION information of the Station of this Node
                - father: NODE (see Node definition) of his father
        """
        
        self.station = station      # STATION information of the Station of this Node
        self.g = 0                  # REAL cost - depending on the type of preference -
                                    # to get from the origin to this Node
        self.h = 0                  # REAL heuristic value to get from the origin to this Node
        self.f = 0                  # REAL evaluate function
        if father ==None:
            self.parentsID=[]
        else:
            self.parentsID = [father.station.id]
            self.parentsID.extend(father.parentsID)         # TUPLE OF NODES (from the origin to its father)
        self.father = father        # NODE pointer to his father
        self.time = 0               # REAL time required to get from the origin to this Node
                                    # [optional] Only useful for GUI
        self.num_stopStation = 0    # INTEGER number of stops stations made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.walk = 0               # REAL distance made from the origin to this Node
                                    # [optional] Only useful for GUI
        self.transfers = 0          # INTEGER number of transfers made from the origin to this Node
                                    # [optional] Only useful for GUI
        
    
    def getDistance(self, origin, destination):
        """
        getDistance: Calculates the Euclidean distance between two stations
        :params
                - origin: PATH. The origin station
                - destination: PATH. The destination station
        :returns
                - distance: INTEGER. The minimum distance between origin and destination        
        """
        distance = sqrt((destination.x - origin.x)**2 + (destination.y - origin.y)**2)
        
        return distance


    def setEvaluation(self):
        """
        setEvaluation:  Calculates the Evaluation Function. Actualizes .f value
       
        """
        self.f = self.g + self.h


    def setHeuristic(self, typePreference, node_destination,city):
        """"
        setHeuristic:   Calculates the heuristic depending on the preference selected
        :params
                - typePreference: INTEGER Value to indicate the preference selected: 
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance 
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: PATH of the destination station
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        """
        if(typePreference is 0):
            self.h = 0
        elif(typePreference is 1):
            self.h = (self.getDistance(self.station, node_destination.station))/city.max_velocity
            if self.station.line is not node_destination.station.line:
                self.h += city.min_transfer
        elif(typePreference is 2):
            self.h = self.getDistance(self.station, node_destination.station)
        elif(typePreference is 3):
            if self.station.line == node_destination.station.line:
                self.h=0
            else:
                self.h=1
        elif typePreference is 4:
            if (self.station.name is node_destination.station.name) or (self.station.line != node_destination.station.line):
                self.h = 0
            else:
                self.h = 1


    def setRealCost(self,  costTable):
        """
        setRealCost:    Calculates the real cost depending on the preference selected
        :params
                 - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
        """

        if self.father is not None:
            if self.father.father is not None:
                for station in costTable:
                    if self.father.station.id is station:
                        for ad_station in costTable[station]:
                            if ad_station is self.station.id:
                                self.g = self.father.g + costTable[station][ad_station] 
            else:
                for station in costTable:
                    if self.father.station.id is station:
                        for ad_station in costTable[station]:
                            if ad_station is self.station.id:
                                self.g = costTable[station][ad_station]
        else:
            self.g = 0
 

def Expand(fatherNode, stationList, typePreference, node_destination, costTable,city):
    """
        Expand: It expands a node and returns the list of connected stations (childrenList)
        :params
                - fatherNode: NODE of the current node that should be expanded
                - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
                - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Null Heuristic
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
                - node_destination: NODE (see Node definition) of the destination
                - costTable: DICTIONARY. Relates each station with their adjacency an their real cost. NOTE that this
                             cost can be in terms of any preference.
                - city: CITYINFO with the information of the city (see CityInfo class definition)
        :returns
                - childrenList:  LIST of the set of child Nodes for this current node (fatherNode)

    """
    
    childrenList = []

    for i in stationList:
        if fatherNode.station.destinationDic.has_key(i.id):
            child = Node(i,fatherNode)
            child.setHeuristic(typePreference, node_destination, city)
            child.setRealCost(costTable)
            child.setEvaluation()
            childrenList.append(child)

    return childrenList


def RemoveCycles(childrenList):
    """
        RemoveCycles: It removes from childrenList the set of childrens that include some cycles in their path.
        :params
                - childrenList: LIST of the set of child Nodes for a certain Node
        :returns
                - listWithoutCycles:  LIST of the set of child Nodes for a certain Node which not includes cycles
    """
    
    listWithoutCycles = childrenList
    repeatedNodes = []
    
    for child in listWithoutCycles:
        father = child.father
        while father is not None:
            if child.station.id is father.station.id:
                if child not in repeatedNodes:
                    repeatedNodes.append(child)
            father = father.father

    for node in repeatedNodes:
        for child in listWithoutCycles:
            if child is node:
                listWithoutCycles.remove(child)

    """
    for child in listWithoutCycles:
        if child in repeatedNodes:
            listWithoutCycles.remove(child)
    """
    
    return listWithoutCycles


def RemoveRedundantPaths(childrenList, nodeList, partialCostTable):
    """
        RemoveRedundantPaths:   It removes the Redundant Paths. They are not optimal solution!
                                If a node is visited and have a lower g in this moment, TCP is updated.
                                In case of having a higher value, we should remove this child.
                                If a node is not yet visited, we should include to the TCP.
        :params
                - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
                - nodeList : LIST of NODES to be visited
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node
        :returns
                - childrenList: LIST of NODES, set of childs without rendundant path.
                - nodeList: LIST of NODES to be visited updated (without redundant paths)
                - partialCostTable: DICTIONARY of the minimum g to get each key (Node) from the origin Node (updated)
    """

    auxList = childrenList[:]
    
    for child in childrenList:
        if partialCostTable.has_key(child.station.id):
            if child.g < partialCostTable[child.station.id]:
                partialCostTable[child.station.id] = child.g
                for node in nodeList:
                    if node.station.id is child.station.id:
                        nodeList.remove(node)
            else:
                for children in childrenList:
                    if child.station.id is children.station.id:
                        auxList.remove(child)
        else:
            partialCostTable.setdefault(child.station.id, child.g)
    
    childrenList = auxList
    
    return childrenList, nodeList, partialCostTable
    


def sorted_insertion(nodeList,childrenList):
    """ Sorted_insertion:   It inserts each of the elements of childrenList into the nodeList.
                            The insertion must be sorted depending on the evaluation function value.
                            
        : params:
            - nodeList : LIST of NODES to be visited
            - childrenList: LIST of NODES, set of childs that should be studied if they contain rendundant path
                                or not.
        :returns
                - nodeList: sorted LIST of NODES to be visited updated with the childrenList included 
    """

    for child in childrenList:
        if not nodeList:
            nodeList.append(child)
        else:
            for index in range(len(nodeList)):
                if (child.f <= nodeList[index].f) and (child not in nodeList):
                    nodeList.insert(index, child)
            if child not in nodeList:
                nodeList.append(child)
                    
    return nodeList
    

def setCostTable( typePreference, stationList,city):
    """
    setCostTable :      Real cost of a travel.
    :param
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)
            - city: CITYINFO with the information of the city (see CityInfo class definition)
    :return:
            - costTable: DICTIONARY. Relates each station with their adjacency an their g, depending on the
                                 type of Preference Selected.
    """

    costTable = {}

    if typePreference == 0:
        for i in city.adjacency.keys():
            costTable[i] = {}
            for j in city.adjacency[i].keys():
                if costTable.has_key(i):
                    costTable[i][j] = 1
    elif typePreference == 1:
        for i in city.adjacency.keys():
            costTable[i] = {}
            for j in city.adjacency[i].keys():
                if costTable.has_key(i):
                    costTable[i][j] = stationList[i-1].destinationDic[j]

    elif typePreference == 2:
        for father, childs in city.adjacency.iteritems():
            costTable[father] = {}
            for adjacentStation in childs:
                if stationList[father - 1].name != stationList[adjacentStation - 1].name:
                    line = stationList[adjacentStation-1].line
                    speed = city.velocity_lines[line-1]
                    time = stationList[father-1].destinationDic[adjacentStation]
                    dist = time * speed
                    costTable[father][adjacentStation] = dist
                else:
                     costTable[father][adjacentStation] = 0

    elif typePreference == 3:
        for i in city.adjacency.keys():
            costTable[i] = {}
            for j in city.adjacency[i].keys():
                if costTable.has_key(i):
                    if stationList[i-1].line == stationList[j-1].line:
                        costTable[i][j] = 0
                    else:
                        costTable[i][j] = 1
    elif typePreference == 4:    
        for i in city.adjacency.keys():
            for j in city.adjacency[i].keys():
                if costTable.has_key(i):
                    if stationList[i-1].name == stationList[j-1].name:
                        costTable[i][j] = 0
                    else:
                        costTable[i][j] = 1
                else:
                    costTable[i] = {}
                    if stationList[i-1].name == stationList[j-1].name:
                        costTable[i][j] = 0
                    else:
                        costTable[i][j] = 1

    return costTable


def coord2station(coord, stationList):
    """
    coord2station :      From coordinates, it searches the closest station.
    :param
            - coord:  LIST of two REAL values, which refer to the coordinates of a point in the city.
            - stationList: LIST of the stations of a city. (- id, destinationDic, name, line, x, y -)

    :return:
            - possible_origins: List of the Indexes of the stationList structure, which corresponds to the closest
            station
    """
    
    possible_origins = []
    bestDistance = float('inf')
    closestStation = None
    
    for station in stationList:
        distance = sqrt((station.x - coord[0])**2 + (station.y - coord[1])**2)
        if distance < bestDistance:
            bestDistance = distance
            closestStation = station

    for station in stationList:
        if closestStation.name == station.name:
            possible_origins.append(station.id - 1)
    
    return possible_origins


def AstarAlgorithm(stationList, coord_origin, coord_destination, typePreference,city,flag_redundants):
    """
     AstarAlgorithm: main function. It is the connection between the GUI and the AStar search code.
     INPUTS:
            - stationList: LIST of the stations of a city. (- id, name, destinationDic, line, x, y -)
            - coord_origin: TUPLE of two values referring to the origin coordinates
            - coord_destination: TUPLE of two values referring to the destination coordinates
            - typePreference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
                                4 - minimum Stops
            - city: CITYINFO with the information of the city (see CityInfo class definition)
            - flag_redundants: [0/1]. Flag to indicate if the algorithm has to remove the redundant paths (1) or not (0)
            
    OUTPUTS:
            - time: REAL total required time to make the route
            - distance: REAL total distance made in the route
            - transfers: INTEGER total transfers made in the route
            - stopStations: INTEGER total stops made in the route
            - num_expanded_nodes: INTEGER total expanded nodes to get the optimal path
            - depth: INTEGER depth of the solution
            - visitedNodes: LIST of INTEGERS, IDs of the stations corresponding to the visited nodes
            - idsOptimalPath: LIST of INTEGERS, IDs of the stations corresponding to the optimal path
            (from origin to destination)
            - min_distance_origin: REAL the distance of the origin_coordinates to the closest station
            - min_distance_destination: REAL the distance of the destination_coordinates to the closest station
            


            EXAMPLE:
            return optimalPath.time, optimalPath.walk, optimalPath.transfers,optimalPath.num_stopStation,
            len(expandedList), len(idsOptimalPath), visitedNodes, idsOptimalPath, min_distance_origin,
            min_distance_destination
    """



