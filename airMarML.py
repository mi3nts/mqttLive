
# 1 Download the data 
# Create a data frame for Airmar Data 

import pandas as pd
import glob
import os
from functools import reduce
from pandas.core.frame import DataFrame

# Wimda   
wimda = glob.glob("D:\mintsData\*\*\*\*\*\*001e0610c0e4*wimda*.csv")
wimda = pd.concat([pd.read_csv(f) for f in wimda ])
wimda['dateTime'] = pd.to_datetime(wimda['dateTime'])
wimda  = wimda[{'dateTime','airTemperature','barrometricPressureBars','relativeHumidity','dewPoint'}]
wimda = wimda.set_index('dateTime').resample('30S').mean()

bme280 = glob.glob("D:\mintsData\*\*\*\*\*\*001e06323a06*bme280*.csv")
bme280 = pd.concat([pd.read_csv(f) for f in bme280 ])
bme280['dateTime'] = pd.to_datetime(bme280['dateTime'])
bme280  = bme280[{'dateTime','temperature','pressure','humidity'}]
bme280 = bme280.set_index('dateTime').resample('30S').mean()

gpsgpgga2 = glob.glob("D:\mintsData\*\*\*\*\*\*001e06323a06*gpsgpgga2*.csv")
gpsgpgga2 = pd.concat([pd.read_csv(f) for f in gpsgpgga2 ])
gpsgpgga2['dateTime'] = pd.to_datetime(gpsgpgga2['dateTime'])
gpsgpgga2 = gpsgpgga2[{'dateTime','latitudeCoordinate','longitudeCoordinate'}]
gpsgpgga2 = gpsgpgga2.set_index('dateTime').resample('30S').mean()

# compile the list of dataframes you want to merge
data_frames = [bme280, wimda, gpsgpgga2]
mintsData = reduce(lambda  left,right: pd.merge(left,right,on=['dateTime'],
                                            how='inner'), data_frames)

# Get rid of non wstc data 
# def gpsCropCoordinatesUTD(TT,latitude,longitude,latRange,longRange):
#     TT= TT(TT.latitudeCoordinate>latitude-abs(latRange),:)
#     # TT= TT(TT.latitudeCoordinate_mintsDataGPSGPGGA2>latitude-abs(latRange),:)
#     # TT= TT(TT.latitudeCoordinate_mintsDataGPSGPGGA2<latitude+abs(latRange),:);
#     # TT= TT(TT.longitudeCoordinate_mintsDataGPSGPGGA2>longitude-abs(longRange),:);
#     # TT= TT(TT.longitudeCoordinate_mintsDataGPSGPGGA2<longitude+abs(longRange),:);

# return TT

def gpsCropCoordinates(mintsData,latitude,longitude,latRange,longRange):
 
    mintsData = mintsData[mintsData.latitudeCoordinate>latitude-abs(latRange)]
    mintsData = mintsData[mintsData.latitudeCoordinate<latitude+abs(latRange)]
    mintsData = mintsData[mintsData.longitudeCoordinate>longitude-abs(longRange)]
    mintsData = mintsData[mintsData.longitudeCoordinate<longitude+abs(longRange)]

    return mintsData;



mintsData = gpsCropCoordinates(mintsData,32.992179, -96.757777,0.0015,0.0015)
print(mintsData)
