import numpy as np
import math
class RiskCalculation():

    def __init__(self):
        self.ObjectWeight = 1
        self.distanceWeightLid = 0.65
        self.distanceWeightCam = 0.35
        self.empty = 0
        self.emptyPercentage = 0


    def distanceRisk(self, dist): ### for the Lidar
        maxDist = 2.0
        minDist = 0.4
        step = (maxDist - minDist) / 100
        risk = (maxDist - dist) / step
        return risk

    def objectRisk(self, object): ## For the camera
        riskClass = RiskDefinition()
        risk = riskClass.object_to_percentage(object)
        return risk

    def SensorFusion(self, riskCam, percentageCam, riskLidar, percentageLidar):

        ##riskCam is an array, the 5 values are going to be used as base for D-S theorem
        ##riskLidar will determine the value of the Risk depending on the Distance
        self.empty = 0
        self.emptyPercentage = 0
        hLidar = self.levelOfRisk(4, riskLidar, percentageLidar)
        mhLidar = self.levelOfRisk(3, riskLidar, percentageLidar)
        mLidar = self.levelOfRisk(2, riskLidar, percentageLidar)
        lLidar = self.levelOfRisk(1, riskLidar, percentageLidar)

        hCam = self.levelOfRisk(4, riskCam, percentageCam)
        mhCam = self.levelOfRisk(3, riskCam, percentageCam)
        mCam = self.levelOfRisk(2, riskCam, percentageCam)
        lCam = self.levelOfRisk(1, riskCam, percentageCam)

        highLevel = hLidar * self.distanceWeightLid + hCam * self.distanceWeightLid
        mhLevel = mhLidar * self.distanceWeightLid + mhCam * self.distanceWeightLid
        medLevel = mLidar * self.distanceWeightLid + mCam * self.distanceWeightLid
        lowLevel = lLidar * self.distanceWeightLid + lCam * self.distanceWeightLid

        level = np.array([0, lowLevel, medLevel, mhLevel, highLevel])
        riskLevel = np.argmax(level)
        riskString = self.numberToString(riskLevel)
        belief = np.max(level)
        # self.emptyPercentage = self.emptyPercentage/self.empty
        plaussability = 1 - ((lowLevel+mhLevel+medLevel+highLevel - belief)/3)

        return riskLevel, belief, plaussability

    def numberToString(self, level):
        if(level == 1):
            return "Low"
        elif(level == 2):
            return "Medium"
        elif(level == 3):
            return "Med-High"
        else:
            return "High"

    def levelOfRisk(self, level, riskArray, percentage):
        counterLevel = 0
        totPercentage = 0
        for x in range(len(riskArray)):
            if(level==riskArray[x]):
                counterLevel+=1
                totPercentage += percentage[x]
        if(counterLevel==0):
            self.empty += 1
            self.emptyPercentage += sum(percentage)/4
        else:
            totPercentage = totPercentage/counterLevel

        return totPercentage

    def riskCatalog(self, risk):
        ## 4 for High | 3 for med-High | 2 for Mid | 1 for Low
        if (risk > 80):
            fuzzRisk = 4
            if(risk>=100):
                percentage = 1
            else:
                percentage = 1-((100-risk)/20)
        elif (risk >=60 and risk<=80):
            fuzzRisk = 3
            if (risk >= 80):
                percentage = 1
            else:
                percentage = 1 - ((80 - risk) / 20)
        elif (risk > 30 and risk<60):
            fuzzRisk = 2
            if (risk >= 59):
                percentage = 1
            else:
                percentage = 1 - ((59 - risk) / 30)
        else:
            fuzzRisk = 1
            if (risk >= 29):
                percentage = 1
            else:
                percentage = 1 - ((29 - risk) / 30)
        return fuzzRisk, percentage

    def prospectTheory(self, level, risk):
        alpha = 0.88
        beta = 0.88
        gamma = 0.61
        lamba = 2.25
        x = level ## the higher the risk, the less gain it should have
        p = risk

        vx = pow(x, alpha)
        # print(vx)
        px = pow(p, gamma)/pow((pow(p, gamma)+pow((1-p), gamma)), gamma)
        # print(px)
        ## analysis
        return vx, px

class RiskDefinition():

    def object_to_percentage(self, argument):
        """Dispatch method"""
        method_name = str(argument)
        # Get the method from 'self'. Default to a lambda.
        method = getattr(self, method_name, lambda: 0)
        # Call the method as we return it
        return method()
    ##Person = Highest risk
    ## Scissors = 90%
    ## Chair = 90%
    ##table = 40%
    ## fork = 10%
    ## book = 10%
    ### All this objects must be modified according to nuclear potential risks. an analysis must be done.

    def laptop(self):##High
        return 100

    def cup(self):##Med-High
        return 60

    def scissors(self):###High
        return 90
 
    def dog(self):#Med-Hi ### will be a cup
        return 50

    def table(self):#Mid
        return 40

    def fork(self):##Low
        return 20

    def book(self):##Low
        return 10
