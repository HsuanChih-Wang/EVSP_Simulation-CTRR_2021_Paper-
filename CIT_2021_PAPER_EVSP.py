
import os
import sys
import optparse
import random

from wang_test_package import SignalPlanObject
from wang_test_package import RSU
from wang_test_package import OBU

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

# 有安裝OBU車輛列表
OBU_Dict = {}
# EV list
EV_list = ['EV_1', 'EV_2', 'EV_3', 'EV_4', 'EV_5', 'EV_6', 'EV_7', 'EV_8', 'EV_9', 'EV_10', 'EV_11', 'EV_12',
           'EV_13', 'EV_14', 'EV_15', 'EV_16', 'EV_17', 'EV_18', 'EV_19', 'EV_20', 'EV_21', 'EV_22', 'EV_23', 'EV_24']

# 路口資料
RSUs = dict()
signalOPTs = dict()


def initialization():
    # 初始化: 新增RSU
    RSU_I1 = RSU.RSU(ID='I1', location=[300, 0], detectionRange=300)
    RSUs.update({'RSU_I1': RSU_I1})
    RSUs['RSU_I1'].initilization()

# contains TraCI control loop
def run(CONTROL_STRATEGY):
    step = 0
    # 號誌設定初始化 SignalPlan Initialization
    initialization()
    #traci.trafficlight.setProgram(tlsID='I1', programID=0)

    while traci.simulation.getMinExpectedNumber() > 0:
        # 迴圈直到所有車輛都已離開路網
        traci.simulationStep()

        print("########################### step = ", step, " ###########################")

        for rsu in RSUs:  # RSUs = ['rsu':'RSU object']
            # vehIDlist = traci.vehicle.getIDList()  # 取出路網中所有車輛ID
            for EV in EV_list:
                result = RSUs[rsu].checkVehExisted(vehID=EV)
                if result:
                    vehX = RSUs[rsu].getVehicleParameters(vehID=EV)
                    if (vehX['nextTLSID'] != None) and (vehX['dist'] <= RSUs[rsu].detectionRange): #確認緊急車前方還有下一個路口
                        traci.vehicletype.setParameter(objID='EmergencyVehicle', param="has.bluelight.device", value="true")
                        #設定speedFactor = 1.5 允許超速
                        traci.vehicle.setSpeedFactor(vehID=vehX['vehID'], factor=1.5)

                        #traci.vehicletype.setParameter(objID='EmergencyVehicle', param="jmDriveAfterRedTime", value="1000")
                        if ((vehX['vehID'] not in OBU_Dict)):
                            roadID_of_obu = traci.vehicle.getRoadID(vehID=vehX['vehID'])
                            targetPhase = roadMapToTargetPhase[roadID_of_obu]
                            # 關閉緊急車安全檢查條件
                            traci.vehicle.setSpeedMode(vehID=vehX['vehID'], sm=32)
                            OBU_Dict.update({vehX['vehID']: OBU.OBU(ID=vehX['vehID'], vehType=vehX['type'], pos=vehX['position'], currentSpeed=vehX['vehSpeed'],
                                             nextTLS=vehX['nextTLSID'], nextTLSIndex=vehX['nextTLSIndex'], nextTLSState=vehX['nextTLSState'], targetPhase=targetPhase, direction=0)})
                        else:
                            print("同一組OBU: %s  已經在OBU list: %s 中，更新相關參數" % (vehX['vehID'], OBU_Dict))
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
                                del OBU_Dict[vehX['vehID']]  # 若 obu 前方已經沒有路口，則刪除該OBU
                                RSUs[rsu].cancelSignalPreemption()  # 取消優先控制
                                # 取消bluelight device
                                traci.vehicletype.setParameter(objID='EmergencyVehicle', param="has.bluelight.device", value="false")
                                # 開啟緊急車安全檢查條件
                                traci.vehicle.setSpeedMode(vehID=vehX['vehID'], sm=31)
                                traci.vehicle.changeLane(vehID=vehX['vehID'], laneIndex=0, duration=200)
                                vehOnRoad = traci.vehicle.getRoadID(vehID=vehX['vehID'])
                                if vehOnRoad in ['E1C1', 'E4C4']:
                                    traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=1.0)
                                else:
                                    traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=-1.0)
                                #traci.vehicle.remove(vehID=vehX['vehID'])
                                #traci.vehicle.changeSublane(vehID=vehX['vehID'], latDist=0)

                    for obu in OBU_Dict:
                        RSUs[rsu].executeSignalPreemption(obu=OBU_Dict[obu], preemptionStrategy=CONTROL_STRATEGY)
                        print("111")

            RSUs[rsu].updateTrafficSignalInformation()
            RSUs[rsu].checkPhaseCommandBuffer()

        print("nowPhaseID = ", traci.trafficlight.getPhase('I1'))
        print("nowState = ", traci.trafficlight.getRedYellowGreenState('I1'))
        print("nextSwitch = ", traci.trafficlight.getNextSwitch('I1'))

        #traci.trafficlight.setProgram(tlsID='I1', programID=6)
        if step == 100:
            print("sss")
        if step == 4200:
            print("dddd")
        if step == 900:
            print("ttt")

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

    route = VC + '/' + EV_level + '/' + LeftLevel + '/' + 'result' + '/' + str(CONTROL_STRATEGY) + '/'

    #====SUMO=====
    options = get_options()

    # check binary
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    randomNum = random.randrange(100, 1000)

    # traci starts sumo as a subprocess and then this script connects and runs
    sumo_start = [sumoBinary, "-c", "CIT_2021_PAPER_EVSP.sumocfg",  #CIT_2021_PAPER_EVSP_VC0.9_EV_High_HighLeft.rou
                  "--route-files", "CIT_2021_PAPER_EVSP_" + VC + "_" + EV_level + "_" + LeftLevel + ".rou.xml",
                  "--tripinfo-output", route + "/" + str(randomNum) + "_" + "CIT_2021_EVSP_tripInfo.xml",
                  "--start"]  # "--random", "--seed", "8"
                # "--device.emissions.probability 1.0"
                # --emission-output", "result/" + str(CONTROL_STRATEGY) + "/CIT_2021_EVSP_emissionInfo.xml",
                # "--full-output", "result/" + str(CONTROL_STRATEGY) + "/CIT_2021_EVSP_fullOutput.xml",

    traci.start(sumo_start)
    traci.vehicletype.setParameter(objID="EmergencyVehicle", param="device.ssm.file", value=route + "/" + str(randomNum) + "_" + "SSM.xml")

    run(CONTROL_STRATEGY=CONTROL_STRATEGY)
