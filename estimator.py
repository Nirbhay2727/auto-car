import util 
from util import Belief, pdf 
from engine.const import Const
import math, collections, random

# Class: Estimator
#----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.
class Estimator(object):

    NUM_PARTICLES = 2000

    def __init__(self, rows, cols):
        self.transProb = util.loadTransProb()
    
        self.belief = util.Belief(rows, cols) 

        self.transProbDict = {}
        for (old, new) in self.transProb:
            if not old in self.transProbDict:
                self.transProbDict[old] = {}
            self.transProbDict[old][new] = self.transProb[(old, new)]

        # Initialize the particles randomly.
        self.particles = dict()
        newParticles = list(self.transProbDict.keys())
        for i in range(self.NUM_PARTICLES):
            particleIndex = int(random.random() * len(newParticles))
            if not newParticles[particleIndex] in self.particles:
                self.particles[newParticles[particleIndex]] = 0
            self.particles[newParticles[particleIndex]] += 1

        self.updateBelief() 
            
    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based onthe distance observation and your current position.
    #
    # - posX: x location of AutoCar 
    # - posY: y location of AutoCar 
    # - observedDist: current observed distance of the StdCar 
    # - isParked: indicates whether the StdCar is parked or moving. 
    #             If True then the StdCar remains parked at its initial position forever.
    # 
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine, 
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################


    def weightedRandomChoice(self,weightDict):
        weights = list()
        elems = list()
        total = 0
        for elem in weightDict:
            weights.append(weightDict[elem])
            elems.append(elem)
            total += weightDict[elem]
        key = random.uniform(0, total)
        runningTotal = 0.0
        chosenIndex = None
        for i in range(len(weights)):
            weight = weights[i]
            runningTotal += weight
            if runningTotal > key:
                chosenIndex = i
                return elems[chosenIndex]

    def updateBelief(self):
        new_belief = util.Belief(self.belief.getNumRows(), self.belief.getNumCols(), 0)
        for (y,x) in self.particles:
            new_belief.setProb(y, x, self.particles[(y,x)])
        new_belief.normalize()
        self.belief = new_belief

    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:
        # BEGIN_YOUR_CODE
        
        for tile in self.particles.keys():
            x = util.colToX(tile[1])
            y = util.rowToY(tile[0])
            initial = self.particles[tile]
            probability = util.pdf(math.sqrt((posX - x) ** 2 + (posY - y) ** 2), Const.SONAR_STD, observedDist)
            posterior = initial * probability
            self.particles[tile] = posterior
        newParticles = collections.Counter()
        for i in range(self.NUM_PARTICLES):
            sample = self.weightedRandomChoice(self.particles)
            newParticles[sample] += 1
        self.particles = newParticles
        self.updateBelief()
        newParticles = collections.Counter()
        for tile in self.particles:
            for i in range(self.particles[tile]):  
                newTile = util.weightedRandomChoice(self.transProbDict[tile])
                newParticles[newTile] += 1
        self.particles = newParticles

        # END_YOUR_CODE
        return

  
    def getBelief(self) -> Belief:
        return self.belief