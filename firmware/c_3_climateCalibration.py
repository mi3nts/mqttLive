# 1 Download the data 
# Create a data frame for Airmar Data 
import pickle
import datetime
import pandas as pd
#import feather
import glob
import os
from collections import OrderedDict
from functools import reduce
from pandas._libs.tslibs import timestamps
from pandas.core.frame import DataFrame
from yaml.events import CollectionStartEvent
from mintsXU4 import mintsProcessing as mP
from mintsXU4 import mintsDefinitions as mD
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression
#import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error

nodeIDs              = mD.mintsDefinitions['nodeIDs']
airMarID             = mD.mintsDefinitions['airmarID']
climateTargets       = mD.mintsDefinitions['climateTargets']
rawPklsFolder        = mD.rawPklsFolder
referencePklsFolder  = mD.referencePklsFolder
mergedPklsFolder     = mD.mergedPklsFolder
modelsPklsFolder     = mD.modelsPklsFolder

dateNow = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
print("Current Time:",dateNow)

for nodeData in nodeIDs:
    print(" ")
    print("=====================MINTS=====================")
    nodeID              = nodeData['nodeID']
    climateSensor       = nodeData['climateSensor']
    sensorDate          = nodeData['climateSensorBegin']
    print("Climate data Calibration for Node: " + nodeID +" with Climate Sensor: " + climateSensor)
    print("running from: " + sensorDate)
    print("-----------------------------------------------")
    try:
        mintsData = pd.read_pickle(mP.getPathGeneric(mergedPklsFolder,nodeID,"climateDataWSTCCurrent","pkl"))
        mintsData = mP.oobClimateCheck(mintsData,nodeID,climateSensor,dateNow,modelsPklsFolder,sensorDate)
        mP.climateCalibration(nodeID,dateNow, mintsData,climateTargets,climateSensor,sensorDate)
    except Exception as e:
        print("[ERROR] Could not publish data, error: {}".format(e))
    
