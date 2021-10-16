# ***************************************************************************
#  mintsXU4
#   ---------------------------------
#   Written by: Lakitha Omal Harindha Wijeratne
#   - for -
#   Mints: Multi-scale Integrated Sensing and Simulation
#      & 
#   TRECIS: Texas Research and Education Cyberinfrastructure Services (TRECIS) NSF Award #2019135
#   The University of Texas at Dallas
# 
#   The authors acknowledge the Texas Research and Education Cyberinfrastructure Services 
#   (TRECIS) Center, # NSF Award #2019135, and The University of Texas at Dallas for 
#   providing {HPC, visualization, database, or grid} resources and support that have 
#   contributed to the research results reported within this project. 
#   URL: https://trecis.cyberinfrastructure.org/
#   ---------------------------------
#   Date: October 1st, 2021
#   ---------------------------------
#   This module is written for generic implimentation of MINTS projects
#   --------------------------------------------------------------------------
#   https://github.com/mi3nts
#   http://utdmints.info/
#   
#   TRECIS URL: https://trecis.cyberinfrastructure.org/
#  ***************************************************************************

from numpy import float64
import serial
import datetime
import os
import csv

import deepdish as dd
#from airMarML import BME280
# from mintsXU4 import mintsLatest as mL
from mintsXU4 import mintsDefinitions as mD
from getmac import get_mac_address
import time
import serial
import pynmea2
from collections import OrderedDict
import netifaces as ni
import math
import pandas as pd
import feather
import glob
from functools import reduce
# from sklearn.linear_model import LinearRegression
# import matplotlib.pyplot as plt
# from sklearn.metrics import r2_score
# from sklearn.metrics import mean_squared_error
# from sklearn.model_selection import train_test_split



macAddress       = mD.macAddress
dataFolder       = mD.dataFolder
rawFolder        = mD.rawFolder
timeSpan         = mD.timeSpan
referenceFolder  = mD.referenceFolder
modelsPklsFolder     = mD.modelsPklsFolder
dataFolderMQTT          = mD.dataFolderMQTT
dataFolderMQTTReference = mD.dataFolderMQTTReference
latestOn       = mD.latestOn
mqttOn         = mD.mqttOn

nodeIDs              = mD.mintsDefinitions['nodeIDs']



def getGPS(nodeID):
    for nodeDataCurrent in nodeIDs:
        nodeIDCurrent              = nodeDataCurrent['nodeID']
        if(nodeID == nodeIDCurrent):
            latitude       = nodeDataCurrent['latitude']
            longitude      = nodeDataCurrent['longitude']
            altitude       = nodeDataCurrent['altitude']

            return latitude,longitude,altitude;

    return "","";

def getSensors(nodeID):
    for nodeDataCurrent in nodeIDs:
        nodeIDCurrent              = nodeDataCurrent['nodeID']
        if(nodeID == nodeIDCurrent):
            climateSensor  = nodeDataCurrent['climateSensor']
            pmSensor       = nodeDataCurrent['pmSensor']

            return climateSensor,pmSensor;

    return "","";

def getWritePathDateCSV(folderIn,nodeID,dateTime,labelIn):
     
    writePath = folderIn+"/"+nodeID+"/"+ \
     str(dateTime.year).zfill(4)  + "/" + str(dateTime.month).zfill(2)+ "/"+str(dateTime.day).zfill(2)+"/"+ \
         "MINTS_"+ nodeID + "_" +labelIn + "_" +\
             str(dateTime.year).zfill(4) + "_" +str(dateTime.month).zfill(2) + "_" +str(dateTime.day).zfill(2) +".csv"
    return writePath;

def c2F(temp):
    return 9/5 * temp + 32     

def b2MB(temp):
    return temp*1000