#!/usr/bin/env python
from random import randrange
import numpy as np

class ExperimentInfo(object):
    def __init__(self,experimentNr=1,PracticeRun=False):
        self.data = self.getInfo(experimentNr,PracticeRun)     

    def getInfo(self,experimentNr,PracticeRun):
        experiment = str(experimentNr) + str(PracticeRun)
        switcher={
            '1False':self.experiment1Real,
            '1True':self.experiment1Practice,
            '2False':self.experiment2Real,
            '3False':self.experiment3Real,

        }
        if experiment in switcher:
            func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
                                                                    .format(experiment))
            self.data = func()
            return func()
        else:
            print("invalid experiment name: {}".format(experiment))

    def experiment1Real(self):
        # testDict = {i: i*i for i in range(10)}
        infoSequence={  'experiment': '1DofLeftRight',
                        'trialNr': range(0,31),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],

                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.09,0.09,0.4],[0.09,0.09,0.4],[0.09,0.09,0.4],
                                    [0.4,0.09,0.09]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],

                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],
                                            [0.0,0.0,0.0,1.0]]
                        }
        return infoSequence
    def experiment1Practice(self):
        infoSequence={  'experiment': '1True',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
    def experiment2Real(self):
        infoSequence={  'experiment': '2False',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.4,0.09,0.4],
                                    [0.4,0.4,0.09],
                                    [0.09,0.4,0.4],[0.09,0.4,0.4]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],                                    
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0]]
                        }
        return infoSequence
    def experiment3Real(self):
        infoSequence={  'experiment': '3False',
                        'trialNr': range(0,3),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.09,0.09,0.4],[0.4,0.09,0.4],
                                    [0.2,0.09,0.4],[0.09,0.09,0.4]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0],                                    
                                            [0.0,0.0,0.0,1.0],[0.0,0.0,0.0,1.0]]
                        }
        return infoSequence
        
    def getShape(self,trialNr,data):
        scales = data['scale'][trialNr]
        orientation = data['orientation'][trialNr]
        return scales, orientation

    def generateRandList(self,minValue, maxValue, step, amount):
        coef = 10000.0
        minValue = int(minValue*coef)
        maxValue = int((maxValue+step)*coef)
        step = int(step*coef)

        randList = []
        stop = False
        while len(randList) < amount and stop==False:
            randomNr = randrange(minValue,maxValue,step)/coef
            # print("random nr", format(randomNr))            
            if not randList:
                randList.append(randomNr)
                continue

            if randomNr not in randList:
                randList.append(randomNr)

            if len(randList) == ((maxValue-minValue)/step):
                stop=True

        return randList

            


    def generateScales(self,minSize, maxSize):
        pass

    def shuffleList(self,lijst,sequence):
        lijst = np.array(lijst)
        newList = lijst[sequence]
        return newList.tolist()

    def create1DofSequence(self,smallMedium,mediumLarge,
                    minAngle, maxAngle ,step, amount):
        sizes1DOF = [smallMedium, mediumLarge, smallMedium, mediumLarge]

        # Angle1 = EI.generateRandList(minAngle, maxAngle, step, amount) #0,15
        # Angle2 = EI.generateRandList(minAngle+20, maxAngle+15, step, amount) # 20,30
        # Angle3 = EI.generateRandList(minAngle+20+15, maxAngle+15+15, step, amount) # 35, 45
        # angles1DOF = [Angle1[0], Angle1[1], Angle2[0], Angle2[1], Angle3[0], Angle3[1]]
        
        angles1DOF = EI.generateRandList(minAngle, maxAngle, step, amount)

        # shuffle zize and angles in some way
        # shuffleSeq = EI.generateRandList(0, 5, 1, 6)
        # shuffleSeq = [int(x) for x in shuffleSeq]
        # sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
        # angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


        sizes1DOF = 5*sizes1DOF 
        angles1DOF = 5*angles1DOF

        shuffleSeq = EI.generateRandList(0, 29, 1, 30)
        shuffleSeq = [int(x) for x in shuffleSeq]
        # sizes1DOF = EI.shuffleList(sizes1DOF,shuffleSeq)
        # angles1DOF = EI.shuffleList(angles1DOF,shuffleSeq)


        # # add starting angle and size to list
        sizes1DOF = [mediumLarge] + sizes1DOF
        angles1DOF = [0] + angles1DOF
        print(sizes1DOF)
        print(angles1DOF)
        return sizes1DOF, angles1DOF


if __name__ == "__main__":  
    EI = ExperimentInfo(1,False)

    # create size list ordered
    smallMedium, mediumLarge = [0.12, 0.4]

    # create angle list ordered
    minAngle, maxAngle, step, amount = [-90, 90, 180/4, 4]

    s1,a1 = EI.create1DofSequence(smallMedium, mediumLarge,minAngle, maxAngle, step, amount)
    print(len(s1))
    print(len(a1))
    # print(s1.count(0.4))
    # set(a1)
    # print(a1.count())
    # small change

