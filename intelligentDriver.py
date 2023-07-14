'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError
import math 
import random

# Class: Graph
# -------------
# Utility class
class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

    def getNbrs(self, node):
        nbrs = []
        for edge in self.edges:
            if edge[0][0] == node[0] and edge[0][1] == node[1]:
                if edge[1] not in nbrs:
                    nbrs.append(edge[1])
        return nbrs
# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):

    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout 
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()

    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                edges.append((tile, node))
        return Graph(nodes, edges)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!
        
        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''
        goalPos = (0, 0) # next tile 
        moveForward = True

        currPos = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        # BEGIN_YOUR_CODE 
        # print("currpos", currPos)
        # nextGoal = self.checkPoints[chkPtsSoFar]
        def euclideanDistance(x1, y1, x2, y2):
            return math.sqrt((x1-x2)*2 + (y1-y2)*2)

        def manhattanDistance(x1, y1, x2, y2):
            return abs(x1-x2) + abs(y1-y2)
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        print("RC->", self.layout.getBeliefRows(), self.layout.getBeliefCols())
        dxList = [-1, 0, +1]
        dyList = [-1, 0, +1]

        def isBlockInVicinity(xPos, yPos): # True if block nearby
            for dx in dxList:
                for dy in dyList:
                    x = xPos + 1.3*dx*Car.LENGTH 
                    y = yPos + 1.3*dy*Car.LENGTH 
                    row = util.yToRow(y); col = util.xToCol(x)
                    if (row, col) not in self.worldGraph.nodes:
                        return True 
            return False 

        ## Check if there is 5% chance 
        def isCarInVicinity(row, col): # True if there is a car nearby
            for dx in dxList:
                for dy in dyList:
                    r = row + dx 
                    c = col + dy
                    p = 0
                    for belief in beliefOfOtherCars:
                        p += belief.getProb(r, c)
                    if p > 0.02 * len(beliefOfOtherCars):
                        return True
            return False 
    
        def isBlockCell(row, col):
            if (row, col) not in self.worldGraph.nodes:
                return True
            return False
        def getProbCar(row, col):
            p = 0
            for belief in beliefOfOtherCars:
                p += belief.getProb(row, col)
            return p
        def numberOfBlocksInvicinity(row, col):
            cnt = 0
            for dx in dxList:
                for dy in dyList:
                    x = row + dx 
                    y = col + dy  
                    if (row, col) not in self.worldGraph.nodes:
                        cnt += 1
            return cnt 
        nextGoal = self.checkPoints[chkPtsSoFar]
        x0 = currPos[0]; x1 = currPos[1]
        
        currNode = (util.yToRow(x1), util.xToCol(x0) )

        minDist = 1e9  # set to inf
        print("Curr", currNode)
        goalNode = None 
        for dx in dxList:
            for dy in dyList:
                if dx != 0 or dy != 0:
                    x = currPos[0] + dx*1.5*Car.LENGTH 
                    y = currPos[1] + dy*1.5*Car.LENGTH 
                    row = util.yToRow(y); col = util.xToCol(x)
                    if (row , col) in self.worldGraph.nodes and row < numRows and col < numCols:
                        dist =  manhattanDistance(row, col, nextGoal[0], nextGoal[1])
                        print("ROW->", row, "COL->", col)

                        ## Check this condn
                        if row < numRows-1 and col < numCols-1 and not isCarInVicinity(row, col) and not isBlockInVicinity(x, y): # less than 5% chance that a car is there
                            if dist <= minDist:
                                goalNode = (row, col)
                                minDist = dist 

        
        if goalNode == None:
            print("Goal Node not Decided")
            cond = currNode[0] > 0 and currNode[1] > 1 and currNode[0] < numRows - 1 and currNode[1] < numCols - 1
            if cond and isCarInVicinity(currNode[0], currNode[1]):
                print("Car in Vicinity")
                minProb = len(beliefOfOtherCars)
                for dx in dxList:
                    for dy in dyList:
                        tempX = currPos[0] + dx*1.5*Car.LENGTH
                        tempY = currPos[1] + dy*1.5*Car.LENGTH
                        tempNode = ( util.yToRow(tempY),util.xToCol(tempX) )
                        condn = tempNode[0] < numRows -1 and tempNode[1] < numCols-1 and tempNode[0] >= 1 and tempNode[1] >=1
                        if condn and tempNode in self.worldGraph.nodes and  minProb > getProbCar(tempNode[0], tempNode[1]) and not isBlockCell(tempNode[0], tempNode[1]):
                            minProb = getProbCar(tempNode[0], tempNode[1])
                            goalPos = (tempX, tempY)
                            moveForward = True
            else:
                print("Block in vicinity not Car")
                for dx in dxList:
                    for dy in dyList:
                        # print("Error aane wala hai ?")
                        row = currNode[0]+dx
                        col = currNode[1]+dy  
                        condn = row >= 0 and col >= 0  and row < numCols-1 and col  < numCols-1                   
                        minDist = 1e9
                        cntBlk = 10 
                        if condn  and (dx != 0 or dy != 0) and (row, col) in self.worldGraph.nodes and not isBlockCell(row, col) and not isCarInVicinity(row, col):
                                dist = manhattanDistance(row, col, nextGoal[0], nextGoal[1])
                                cnt = numberOfBlocksInvicinity(row, col)
                                print(cnt)
                                if cnt < cntBlk or (cnt == cntBlk and minDist > dist):
                                    cntBlk = cnt
                                    minDist = dist
                                    goalPos = (util.colToX(col), util.rowToY(row))
                                    moveForward = True
                if goalNode == None:
                    moveForward = False 
        else:
            goalPos = (util.colToX(goalNode[1]), util.rowToY(goalNode[0]))


        # END_YOUR_CODE
        return goalPos, moveForward

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]
       
        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1
        return actions