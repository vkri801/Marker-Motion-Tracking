#Class for creating video captures

import numpy as np
import glob
import math

class Filter:
    
    def __init__(self, SampleLength):

        self.samples = np.zeros(shape=(SampleLength))
        self.counter = 0
        self.initialisation = 0
        self.average = 0
        self.N = SampleLength

    ###------------------ ---------------------------

    def Processing(self, RawData):
        
        self.samples[self.counter] = RawData
        self.counter = self.counter + 1

        if self.counter > (self.N-1):
            self.counter = 0
            self.initialisation = 1

        if self.initialisation == 0:
            self.average = RawData
        else:
            #self.average = np.sum(self.samples)/self.N
            self.average = round(np.sum(self.samples)/self.N,6)
   ###------------------ ---------------------------
            
        

        
            

        



    
