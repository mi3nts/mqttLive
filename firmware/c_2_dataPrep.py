
# 1 Download the data 
# Create a data frame for Airmar Data 
import pickle
import datetime
import pandas as pd
#import feather
import glob
from collections import OrderedDict
import os
from functools import reduce
from pandas._libs.tslibs import timestamps
from pandas.core.frame import DataFrame
from mintsXU4 import mintsProcessing as mP
from mintsXU4 import mintsDefinitions as mD

nodeIDs              = mD.mintsDefinitions['nodeIDs']
airMarID             = mD.mintsDefinitions['airmarID']
rawPklsFolder        = mD.rawPklsFolder
referencePklsFolder  = mD.referencePklsFolder
mergedPklsFolder     = mD.mergedPklsFolder

## For Testing Purposes
# BME280  = mP.superReader("001e06323a06","BME280");
# print(BME280)

#YXXDR     = mP.superReader(airMarID,"YXXDR")
#pd.to_pickle(YXXDR ,mP.getPathGeneric(referencePklsFolder,airMarID,"YXXDR","pkl") )

#WIMDA     = mP.superReader(airMarID,"WIMDA")
#pd.to_pickle(WIMDA ,mP.getPathGeneric(referencePklsFolder,airMarID,"WIMDA","pkl") )

WIMDA  = pd.read_pickle(mP.getPathGeneric(referencePklsFolder,airMarID,"WIMDA","pkl"))
YXXDR  = pd.read_pickle(mP.getPathGeneric(referencePklsFolder,airMarID,"YXXDR","pkl"))

print("--------------")

for nodeData in nodeIDs:
    nodeID        = nodeData['nodeID']
    climateSensor = nodeData['climateSensor']
    print("=====================MINTS=====================")
    print("Prepearing Climate data for Node: " + nodeID +" with Climate Sensor: " + climateSensor)
    print("-----------------------------------------------")
    mP.climateDataPrep(nodeData,nodeID,climateSensor,WIMDA,YXXDR,mergedPklsFolder)

