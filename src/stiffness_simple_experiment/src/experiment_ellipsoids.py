#!/usr/bin/env python

class ExperimentInfo(object):
    def __init__(self,experimentNr=1,PracticeRun=False):
        self.data = self.getInfo(experimentNr,PracticeRun)     

    def getInfo(self,experimentNr,PracticeRun):
        experiment = str(experimentNr) + str(PracticeRun)
        switcher={
            '1False':self.experiment1Real,
            '1True':self.experiment1Practice,
            '2False':self.experiment2Real,

        }
        if experiment in switcher:
            func = switcher.get(experiment, lambda: "invalid experiment definition: {}"
                                                                    .format(experiment))
            self.data = func()
            return func()
        else:
            print("invalid experiment name: {}".format(experiment))

    def experiment1Real(self):
        infoSequence={  'experiment': '1False',
                        'trialNr': range(0,2),
                        # tussen 0.0849 - 0.56557
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
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
                        'scale': [  [0.15,0.09,0.09],[0.2,0.15,0.09],
                                    [0.4,0.1,0.1]],
                        'orientation': [    [0.0,0.0,0.0,1.0],[0.3826834, 0.0, 0.0, 0.9238795],
                                            [0.0, 0.5, 0.0, 0.8660254]]
                        }
        return infoSequence
        
    def getShape(self,trialNr,data):
        scales = data['scale'][trialNr]
        orientation = data['orientation'][trialNr]
        return scales, orientation