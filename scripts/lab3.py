#!/usr/bin/env python

import rospy
import math
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

##-----------Globals------------##
ocGrid = OccupancyGrid()
ocGridMeta = MapMetaData()
startNode = None
goalNode = None
closedset = list()
openset = list()
expandGrid = GridCells()
travelPath = list()
TF2buffer = tf2_ros.buffer.Buffer()
position = Point()
aStarComplete = 0


def mapCallback(OccupancyGrid):
    print 'Got Occupancy Map'

    global expandGrid
    global ocGridMeta
    global ocGrid
    expandGrid = GridCells()
    ocGridMeta = OccupancyGrid.info
    ocGrid = [[0 for _ in range(OccupancyGrid.info.height)] for _ in range(OccupancyGrid.info.width)] #Generate the ocgrid as a 2-d array
    for row in range(OccupancyGrid.info.height):
        for col in range(OccupancyGrid.info.width):
            i = ( row * OccupancyGrid.info.width) + col
            if( OccupancyGrid.data[i] >= 30):  #if there is a 60% propabability of obsticle
                ocGrid[col][row] = 1 #flag
                expandObsticals(col, row) #expand the discovered obstical
            elif OccupancyGrid.data[i] == -1:  #if the cell is unknown
                ocGrid[col][row] = 0 #flag
                
    expandGrid.cell_height = ocGridMeta.resolution 
    expandGrid.cell_width = ocGridMeta.resolution
    expandGrid.header.frame_id = 'map'
    ExpandPub.publish(expandGrid)
    global position
    global startNode
    x = position.x
    y = position.y
    x = int(round((x - ocGridMeta.origin.position.x)/ocGridMeta.resolution))
    y = int(round((y - ocGridMeta.origin.position.y)/ocGridMeta.resolution))
    print "Got a start Position"
    startNode =  node(x, y,None,0,0)
    if goalNode != None:
        aStar(startNode, goalNode)
    print 'import done'
                

def expandObsticals(col,row):
    global ocGridMeta
    global ocGrid
    global expandGrid
    radiusRobot = 0.22 #in meters
    numBlocksPerRadius = int(math.ceil(radiusRobot/ocGridMeta.resolution))
    for x in range(col - numBlocksPerRadius, col + numBlocksPerRadius + 1):
        for y in range(row - numBlocksPerRadius, row + numBlocksPerRadius + 1):
            if x >= 0 and y >= 0 and x < len(ocGrid) and y < len(ocGrid[0]) and ocGrid[x][y] != 1:
                ocGrid[x][y] = 1
                expandGrid.cells.append(Point(ocGridMeta.resolution * x + ocGridMeta.origin.position.x + ocGridMeta.resolution/2,
                                              
                                              ocGridMeta.resolution * y + ocGridMeta.origin.position.y + ocGridMeta.resolution/2,
                                              0))    
    
#generate a node for the start position    
def startNodeCallback(PoseWithCovarianceStamped):
    global ocGridMeta
    global startNode
    x = PoseWithCovarianceStamped.pose.pose.position.x
    y = PoseWithCovarianceStamped.pose.pose.position.y
    x = int(round((x - ocGridMeta.origin.position.x)/ocGridMeta.resolution))
    y = int(round((y - ocGridMeta.origin.position.y)/ocGridMeta.resolution))
    print "Got a start Position"
    startNode =  node(x, y,None,0,0)
    if goalNode != None:
        aStar(startNode, goalNode)

#generate a node for the goal position
def goalNodeCallback(PointStamped):
    global ocGridMeta
    global goalNode
    x = PointStamped.point.x
    y = PointStamped.point.y
    x = int(round((x - ocGridMeta.origin.position.x)/ocGridMeta.resolution))
    y = int(round((y - ocGridMeta.origin.position.y)/ocGridMeta.resolution))
    print "Got a end Position"
    goalNode = node(x, y,None,0,0) 
    if startNode != None:
        aStar(startNode, goalNode)
        print "A* Complete"

#Publish a map for the prelab
def publishMap():
    meta = MapMetaData()
    meta.width = 10 
    meta.height = 10
    meta.resolution = 0.2
    for c in range(-1,100):
        map = OccupancyGrid()
        map.header.frame_id = 'map'
        for i in range(0,100):
            val = c + i
            if val > 100:
                val = val - 101
            map.data.append(val)
        map.info = meta
        mapPub.publish(map)
        publishGrid()
        
#publish a grid for a set of nodes
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


class node:
    def __init__(self, xIndex, yIndex, parent, g_score, f_score):
        self.xIndex = xIndex
        self.yIndex = yIndex
        #In meters
        self.xPosition = ocGridMeta.resolution * xIndex + ocGridMeta.origin.position.x + ocGridMeta.resolution/2
        self.yPosition = ocGridMeta.resolution * yIndex + ocGridMeta.origin.position.y + ocGridMeta.resolution/2
        #as unique ids
        self.parent = parent
        self.g_score = g_score
        self.f_score = f_score
        self.name = ocGridMeta.width*yIndex + xIndex
    
    def __repr__(self):
        return repr((self.xPosition, self.yPosition))#, self.parent, self.g_score, self.f_score))
    
# calculate heuristic cost
def heuristic_cost_estimate(start, end):
    startX = start.xIndex
    startY = start.yIndex
    endX = end.xIndex
    endY = end.yIndex
    return abs(endX - startX) + abs(endY - startY)

# reconstruct the path
def reconstruct_path(node, list):
    list.append(node)
    if node.parent != None:
        for nodeX in closedset:
            if nodeX.name == node.parent:
                return reconstruct_path(nodeX, list)
    else:
        return list

#list all the neighbor nodes
def neighbor_nodes(centerNode):
    global ocGrid
    global openset
    global closedset
    neighbors = list()
    for x in range(-1, 2):
        for y in range(-1,2):
            if x == 0 and y == 0:
                continue
            xpos = x + centerNode.xIndex
            ypos = y + centerNode.yIndex
            if xpos >= 0 and ypos >= 0 and xpos < len(ocGrid) and ypos < len(ocGrid[0]):
                if ocGrid[xpos][ypos] == 1:
                    continue
                else:
                    found = 0
                    nodeMeSilly = node(xpos,ypos,None,0,0)
                    for nodes in openset:
                        if nodes.name == nodeMeSilly.name:
                            neighbors.append(nodes)
                            found = 1
                            continue
                    for nodes in closedset:
                        if nodes.name == nodeMeSilly.name:
                            neighbors.append(nodes)
                            found = 1
                            continue

                    if found == 0:
                        neighbors.append(nodeMeSilly)
    #publishGrid(neighbors, NeighPub)
    return neighbors
                
def dist_between(start, end):
    if start.xIndex != end.xIndex and start.yIndex != end.yIndex:
        return 1.414
    else:
        return 1
    
        
def aStar(start,goal):
    startNode = node(start.xIndex, start.yIndex, None, 0, start.g_score +  heuristic_cost_estimate(start,goal))
    global closedset
    global openset
    global aStarComplete
    global travelPath
    aStarComplete = 0
    closedset = list()
    openset = list()
    openset.append(startNode)
    
    while len(openset) > 0:
        openset.sort(key=lambda node: node.f_score)
        current = openset[0]
        
        
        if current.xIndex == goal.xIndex and current.yIndex == goal.yIndex:
            path = reconstruct_path(current, list())
            print 'Path Complete'
            publishGrid(path, PathPub)
            travelPath = path
            travelPath.reverse()
            aStarComplete = 1
            return path
        
        openset.pop(0)
        closedset.append(current)
        publishGrid(closedset, ClosedPub)
        publishGrid(openset, OpendPub)
        
        for neighbor in neighbor_nodes(current):
            tentative_g_score = current.g_score + dist_between(current, neighbor)
            tentative_f_score = tentative_g_score + heuristic_cost_estimate(neighbor, goal)
            
            if neighbor in closedset and tentative_f_score >= neighbor.f_score:
                continue
            if neighbor not in openset or tentative_f_score < neighbor.f_score:
                neighbor.parent = current.name
                neighbor.g_score = tentative_g_score
                neighbor.f_score = tentative_f_score
                if neighbor not in openset:
                    openset.append(neighbor)
    return -1

    
def distanceToPoint(nodeX):
    global position
    return math.sqrt(math.pow(abs(nodeX.xPosition - position.x),2) + math.pow(abs(nodeX.yPosition - position.y),2))

def navigateToGoal():
    global travelPath
    global position
    print "navigating"
    theta = 0
    for ind, nodeX in enumerate(travelPath): 
      
        print ind
        while distanceToPoint(nodeX) > 0.075:
            #print distanceToPoint(nodeX)
            xcurr = position.x
            ycurr = position.y
        
            xnex = nodeX.xPosition
            ynex = nodeX.yPosition
                
            xdiff = xnex - xcurr
            ydiff = ynex - ycurr
        
            theta = math.atan2(ydiff, xdiff)
            velocPub.publish(0.05)
            anglePub.publish(theta)
    #end for loop
    velocPub.publish(0.0)
    


def callback(data):
    #Get Position
    global position
    position = data.pose.pose.position
            

if __name__ == '__main__':

    rospy.init_node('lab3')
    mapPub = rospy.Publisher('map', OccupancyGrid)
    GridPub = rospy.Publisher('map_smile', GridCells)
    OpendPub = rospy.Publisher('map_openlist', GridCells)
    ClosedPub = rospy.Publisher('map_closedlist', GridCells)
    PathPub = rospy.Publisher('map_path', GridCells)
    NeighPub = rospy.Publisher('map_neighbors', GridCells)
    ExpandPub = rospy.Publisher('map_expanded', GridCells)
    velocPub = rospy.Publisher('set_speed', Float64)
    anglePub = rospy.Publisher("set_angle", Float64)
    
    rospy.Subscriber("map", OccupancyGrid, mapCallback)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, startNodeCallback)
    rospy.Subscriber("/clicked_point", PointStamped, goalNodeCallback)
    rospy.Subscriber("shifted_Odom", Odometry, callback)

    
        
    rospy.sleep(1)

    while not rospy.is_shutdown():
        #publishMap()
        if startNode != None and goalNode != None and aStarComplete == 1:
            navigateToGoal() #not working
        else:
            velocPub.publish(0.0)
            #rospy.spin()