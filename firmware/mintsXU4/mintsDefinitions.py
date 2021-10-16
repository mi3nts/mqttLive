
from getmac import get_mac_address
import serial.tools.list_ports
import yaml


tlsCert                   = "/etc/ssl/certs/ca-certificates.crt"     # The path of your TLS cert


#  -------------------------- 

latestOn                  = False

# For MQTT 
mqttOn                    = True
mqttCredentialsFile      = 'mintsXU4/credentials.yml'
sensorNodesFile          = 'sensorNodes.yml'
droneFile                = 'drone.yml'
centralNodesFile         = 'centralNodes.yml'
otterFile                = 'otter.yml'
carRoofFile              = 'carRoof.yml'
utdNodesFile             = 'mintsXU4/currentNodes.yml'
mockNodesFile            = 'sensorNodesMock.yml'
mqttBroker               = "mqtt.circ.utdallas.edu"
mintsDefinitionsFile     = 'mintsXU4/mintsDefinitions.yaml'
mqttPort                 = 8883  # Secure port
# senderNodes            = yaml.load(open(sensorNodesFile))
mintsDefinitions         = yaml.load(open('mintsXU4/mintsDefinitions.yaml'))
# utdNodes                 = yaml.load(open('mintsXU4/utdNodes.yaml'))



def findMacAddress():
    macAddress= get_mac_address(interface="eth0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="docker0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    macAddress= get_mac_address(interface="enp1s0")
    if (macAddress!= None):
        return macAddress.replace(":","")

    return "xxxxxxxx"

macAddress                = findMacAddress()
dataFolder                = mintsDefinitions['dataFolder']
rawFolder                 = dataFolder    + "/raw"
referenceFolder           = dataFolder    + "/reference"
rawPklsFolder             = dataFolder    + "/rawPkls"
referencePklsFolder       = dataFolder    + "/referencePkls"
mergedPklsFolder          = dataFolder    + "/mergedPkls"
modelsPklsFolder          = dataFolder    + "/modelsPkls"
liveFolder                = dataFolder    + "/liveUpdate/results"


# Change Accordingly  
dataFolderMQTTReference   = dataFolder + "/referenceMQTT"  # The path of your MQTT Reference Data 
dataFolderMQTT            = dataFolder + "/rawMQTT"        # The path of your MQTT Raw Data 
dataFolderReference       = dataFolder + "/reference"
dataFolderMQTTCalib       = dataFolder + "/calibratedMQTT"
timeSpan                  = mintsDefinitions['timeSpan']


print()
print("----MINTS Definitions-----")
print("Data Folder                : {0}".format(dataFolder))
print("Raw Folder                 : {0}".format(rawFolder))
print("Reference Folder           : {0}".format(referenceFolder))
print("Time Span                  : {0}".format(timeSpan))
print("Data Folder Ref            : {0}".format(dataFolder))
print("Mac Address                : {0}".format(macAddress))
print("Latest On                  : {0}".format(latestOn))
print("MQTT On                    : {0}".format(mqttOn))
print("MQTT Credentials File      : {0}".format(mqttCredentialsFile))
print("MQTT Broker and Port       : {0}, {1}".format(mqttOn,mqttPort))
# print("Sensor Nodes File          : {0}".format(sensorNodesFile))


