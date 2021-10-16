#!/bin/bash -l

python3 c_1_pythonSyncher.py mintsXU4/mintsDefinitions.yaml 
sleep 5 
python3 c_2_dataPrep.py
sleep 5 
python3 c_3_climateCalibration.py




