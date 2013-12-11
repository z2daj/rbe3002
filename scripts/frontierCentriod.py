#!/usr/bin/env python

import rospy
import math
import numpy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray

frontierNodes = list()
frontierGroups = list()
ocGridMeta = MapMetaData()
ocGrid = OccupancyGrid()

class node:
    global ocGridMeta
    def __init__(self, xIndex, yIndex):
        self.xIndex = xIndex
        self.yIndex = yIndex
        #In meters
        self.xPosition = ocGridMeta.resolution * xIndex + ocGridMeta.origin.position.x + ocGridMeta.resolution/2
        self.yPosition = ocGridMeta.resolution * yIndex + ocGridMeta.origin.position.y + ocGridMeta.resolution/2
        
        def __repr__(self):
            return repr((self.xIndex, self.yIndex))#, self.parent, self.g_score, self.f_score))
        
def mapCallback(OccupancyGrid):
    print 'Got Occupancy Map'

    global ocGridMeta
    global ocGrid
    expandGrid = GridCells()
    ocGridMeta = OccupancyGrid.info
    ocGrid = [[0 for _ in range(OccupancyGrid.info.height)] for _ in range(OccupancyGrid.info.width)] #Generate the ocgrid as a 2-d array
    for row in range(OccupancyGrid.info.height):
        for col in range(OccupancyGrid.info.width):
            i = ( row * OccupancyGrid.info.width) + col
            if( OccupancyGrid.data[i] >= 60):  #if there is a 60% propabability of obsticle
                ocGrid[col][row] = 1 #flag
                expandObsticals(col, row) #expand the discovered obstical
            elif OccupancyGrid.data[i] == -1:  #if the cell is unknown
                ocGrid[col][row] = -1 #flag
    frontierFinder(ocGrid) 
                
def expandObsticals(col,row):
    global ocGridMeta
    global ocGrid
    radiusRobot = 0.22 #in meters
    numBlocksPerRadius = int(math.ceil(radiusRobot/ocGridMeta.resolution))
    for x in range(col - numBlocksPerRadius, col + numBlocksPerRadius + 1):
        for y in range(row - numBlocksPerRadius, row + numBlocksPerRadius + 1):
            if x >= 0 and y >= 0 and x < len(ocGrid) and y < len(ocGrid[0]) and ocGrid[x][y] != 1:
                ocGrid[x][y] = 1
    
def publishGrid(nodeList, rubMyPub):
    global ocGridMeta
    nodeGrid = GridCells()
    nodeGrid.cell_height = ocGridMeta.resolution
    nodeGrid.cell_width = ocGridMeta.resolution
    nodeGrid.header.frame_id = 'map'
    for nodes in nodeList:
        point = Point()
        point.x = nodes.xPosition
        point.y = nodes.yPosition
        point.z = 0
        nodeGrid.cells.append(point)
    rubMyPub.publish(nodeGrid)
        
            
def frontierFinder(data):
    global frontierNodes
    global frontierGroups
    print "finding frontiers"
    for x_coord in range(0, len(data)):
        for y_coord in range(0, len(data[0])):
            if data[x_coord][y_coord] == -1:
                foundOpenNeighbor = 0
                try:
                    if data[x_coord+1][y_coord] == 0:
                        foundOpenNeighbor = 1
                    elif data[x_coord][y_coord+1] == 0:
                        foundOpenNeighbor = 1
                    elif data[x_coord-1][y_coord] == 0:
                        foundOpenNeighbor = 1
                    elif data[x_coord][y_coord-1] == 0:
                        foundOpenNeighbor = 1
                except IndexError:
                    foundOpenNeighbor = 0
                if foundOpenNeighbor == 1:
                    newnode = node(x_coord, y_coord)
                    frontierNodes.append(newnode)
            
    publishGrid(frontierNodes, frontierPub)
     
    while len(frontierNodes) > 0:
        frontierGroup = findFrontierGroup(frontierNodes.pop(), list())
        print "adding group"
        frontierGroups.append(frontierGroup)
        
    frontierGroup = max(frontierGroups, key=(lambda(x): len(x)))
    publishGrid(frontierGroup, finalFrontierPub)
    rospy.sleep(2)
    print "publishing"
    pubSetPoint.publish(findCentroid(frontierGroup))
        
    
def findFrontierGroup(Node, List):
    global frontierNodes
    print "Grouping"
    x_coord = Node.xIndex
    y_coord = Node.yIndex
    List.append(Node)
    for x,y in [(x_coord+i, y_coord+j) for i in (-1,0,1) for j in (-1,0,1) if i != 0 or j != 0]:
        for indx, node in enumerate(frontierNodes):
            if node.xIndex == x and node.yIndex == y:
                print "found a frontier Node"
                List = List + findFrontierGroup(frontierNodes.pop(indx), List)
    return List

def findCentroid(listNodes):
    global ocGridMeta
    x = list()
    y = list()
    num = len(listNodes)
    for Node in listNodes:
        x.append(Node.xPosition)
        y.append(Node.yPosition)
    
    p = PointStamped()
    p.header.frame_id = "map"
    p.point.x = numpy.mean(x)
    p.point.y = numpy.mean(y)
    p.point.z = 0
    return p

if __name__ == '__main__':

    rospy.init_node('frontierCentroid')

    rospy.Subscriber("map", OccupancyGrid, mapCallback)
    
    pubSetPoint = rospy.Publisher("/clicked_point", PointStamped)
    frontierPub = rospy.Publisher('/frontier', GridCells)
    finalFrontierPub = rospy.Publisher('/final_frontier', GridCells)


    while not rospy.is_shutdown():
        rospy.spin()