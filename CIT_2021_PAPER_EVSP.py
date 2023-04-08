
import os
import sys
import optparse
import random

from package import RSU
from package import OBU

roadMapToTargetPhase = {'J2E3': 6, 'N1E2': 0, 'C1E1': 6, 'C4E4': 0}
from scipy import stats

# we need to import some python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # Checks for the binary in environ vars
import traci


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


OBU_Dict = {}
# EV list
EV_list = ['EV_1', 'EV_2', 'EV_3', 'EV_4', 'EV_5', 'EV_6', 'EV_7', 'EV_8', 'EV_9', 'EV_10', 'EV_11', 'EV_12',
           'EV_13', 'EV_14', 'EV_15', 'EV_16', 'EV_17', 'EV_18', 'EV_19', 'EV_20', 'EV_21', 'EV_22', 'EV_23', 'EV_24']
RSUs = dict()
signalOPTs = dict()


def initialization():
    # RSU Initialization
    RSU_I1 = RSU.RSU(ID='I1', location=[300, 0], detectionRange=300)
    RSUs.update({'RSU_I1': RSU_I1})
    RSUs['RSU_I1'].initilization()

# contains TraCI control loop
def run(CONTROL_STRATEGY):
    step = 0
    # SignalPlan Initialization
    initialization()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        print("########################### step = ", step, " ###########################")

        for rsu in RSUs:  # RSUs = ['rsu':'RSU object']
            for EV in EV_list:
                result = RSUs[rsu].checkVehExisted(vehID=EV)
                if result:
                    vehX = RSUs[rsu].getVehicleParameters(vehID=EV)
                    #Make sure that there still has at least one intersection in front the OBU
                    # Condition 1: Check whether the EV has passed the intersectino or not 確認是否已通過路口
                    # Condition 2: 檢查是否有在RSU通訊範圍內
                    if (vehX['nextTLSID'] != None) and (vehX['dist'] <= RSUs[rsu].detectionRange):
                        traci.vehicletype.setParameter(objID='EmergencyVehicle', param="has.bluelight.device", value="true")
                        #SpeedFactor = 1.5 -> speeding is allowed
                        traci.vehicle.setSpeedFactor(typeID=vehX['vehID'], factor=1.5)

                        #traci.vehicletype.setParameter(objID='EmergencyVehicle', param="jmDriveAfterRedTime", value="1000")
                        if ((vehX['vehID'] not in OBU_Dict)):
                            roadID_of_obu = traci.vehicle.getRoadID(vehID=vehXㄛ['vehID'])
                            targetPhase = roadMapToTargetPhase[roadID_of_obu]
                            # Disable the safety check rules (sm=32)
                            traci.vehicle.setSpeedMode(vehID=vehX['vehID'], sm=32)
                            OBU_Dict.update({vehX['vehID']: OBU.OBU(ID=vehX['vehID'], vehType=vehX['type'], pos=vehX['position'], currentSpeed=vehX['vehSpeed'],
                                             nextTLS=vehX['nextTLSID'], nextTLSIndex=vehX['nextTLSIndex'], nextTLSState=vehX['nextTLSState'], targetPhase=targetPhase, direction=0)})
                        else:
                            print("OBU: %s  is existed in OBU list: %s -> UPDATE ITS PARAMETERS" % (vehX['vehID'], OBU_Dict))
                            OBU_Dict[vehX['vehID']].setVehType(vehX['type'])
                            OBU_Dict[vehX['vehID']].setPosition(vehX['position'])
                            OBU_Dict[vehX['vehID']].setCurrentSpeed(vehX['vehSpeed'])
                            OBU_Dict[vehX['vehID']].setNextTLS(vehX['nextTLSID'])
                            OBU_Dict[vehX['vehID']].setNextTLSState(vehX['nextTLSState'])
                            OBU_Dict[vehX['vehID']].setNextTLSIndex(vehX['nextTLSIndex'])
                            #traci.vehicle.changeLane(vehID=vehX['vehID'], laneIndex=0, duration=999)
                    else:
                        if vehX['vehID'] in OBU_Dict:
                            traci.vehicle.changeLane(vehID=vehX['vehID'], laneIndex=1, duration=200)
                            if traci.vehicle.getSpeed(vehID=vehX['vehID']) >= 13:
                                # If there is no intersection in front of the OBU, then remove this OBU from list
                                del OBU_Dict[vehX['vehID']]
                                RSUs[rsu].cancelSignalPreemption()  # Disable Signal Preemption
                                # Disable bluelight device
                                traci.vehicletype.setParameter(objID='EmergencyVehicle', param="has.bluelight.device", value="false")
                                # Enable the safety check rules (sm=31)
                                traci.vehicle.setSpeedMode(vehID=vehX['vehID'], sm=31)
                                traci.vehicle.changeLane(vehID=vehX['vehID'], laneIndex=0, duration=200)
                                vehOnRoad = traci.vehicle.getRoadID(vehID=vehX['vehID'])
                                if vehOnRoad in ['E1C1', 'E4C4']:
                                    traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=1.0)
                                else:
                                    traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=-1.0)
                                #traci.vehicle.remove(vehID=vehX['vehID'])
                                #traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=0)

                    # 決定優先號誌控制
                    for obu in OBU_Dict:
                        RSUs[rsu].executeSignalPreemption(obu=OBU_Dict[obu], preemptionStrategy=CONTROL_STRATEGY)
                        print("111")
            #RSU更新時制秒數資料
            RSUs[rsu].updateTrafficSignalInformation()
            #RSU 確認commnadBuffer內容
            RSUs[rsu].checkPhaseCommandBuffer()

        print("nowPhaseID = ", traci.trafficlight.getPhase('I1'))
        print("nowState = ", traci.trafficlight.getRedYellowGreenState('I1'))
        print("nextSwitch = ", traci.trafficlight.getNextSwitch('I1'))

        step += 1

    traci.close()
    sys.stdout.flush()

# main entry point
if __name__ == "__main__":

    VC = input("V/C = 1. 0.9 2. 0.5\n")
    if VC == '1':
        VC = 'VC_0.9'
    else:
        VC = 'VC_0.5'

    EV_level = input("EV level -> 1 = High 2 = Medium 3 = Low\n")
    if EV_level == '1':
        EV_level = 'EV_High'
    elif EV_level == '2':
        EV_level = 'EV_Medium'
    elif EV_level == '3':
        EV_level = 'EV_Low'

    LeftLevel = input("Left Level -> 1 = High 2 = Medium 3 = Low\n")
    if LeftLevel == '1':
        LeftLevel = 'HighLeft'
    elif LeftLevel == '2':
        LeftLevel = 'MediumLeft'
    elif LeftLevel == '3':
        LeftLevel = 'LowLeft'

    CONTROL_STRATEGY = int(input("ControlStrategy = "))

    #====SUMO=====
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    randomNum = random.randrange(100, 1000)

    # traci starts sumo as a subprocess and then this script connects and runs

    routeFileRoute = os.path.join('route files', "CIT_2021_PAPER_EVSP_" + VC + "_" + EV_level + "_" + LeftLevel + ".rou.xml")
    tripInfoOutputName = str(randomNum) + "_" + "CIT_2021_EVSP_tripInfo.xml"
    tripInfoOutputRoute = os.path.join('data', VC, EV_level, LeftLevel, 'result', str(CONTROL_STRATEGY))

    sumo_start = [sumoBinary, "-c", "CIT_2021_PAPER_EVSP.sumocfg",  #CIT_2021_PAPER_EVSP_VC0.9_EV_High_HighLeft.rou
                  "--route-files", routeFileRoute,
                  "--tripinfo-output", os.path.join(tripInfoOutputRoute, tripInfoOutputName),
                  "--start"]  # "--random", "--seed", "8"
                # "--device.emissions.probability 1.0"
                # --emission-output", "result/" + str(CONTROL_STRATEGY) + "/CIT_2021_EVSP_emissionInfo.xml",
                # "--full-output", "result/" + str(CONTROL_STRATEGY) + "/CIT_2021_EVSP_fullOutput.xml",

    traci.start(sumo_start)
    traci.vehicletype.setParameter(objID="EmergencyVehicle",
                                   param="device.ssm.file",
                                   value=os.path.join(tripInfoOutputRoute, str(randomNum) + "_" + "SSM.xml"))
                                   #value => set a custom file name

    run(CONTROL_STRATEGY=CONTROL_STRATEGY)
