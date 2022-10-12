import traci
import math
# phaseStrDict = {0: "J1", 1: "J2", 2: "J3", 3: "J4",4: "J5", 5: "J6", 6: "J7", 7: "J8"}
# phaseStrDict_rev = {"J1": 0, "J2":1, "J3":2, "J4":3, "J5":4, "J6":5, "J7":6, "J8":7}
# TLS_index_pair = {0: {"phase": 'J1', "direction": 'South'}, 1: {"phase": 'J1', "direction": 'South'}, 2: {"phase": 'J1', "direction": 'South'},
#                   3: {"phase": 'J6', "direction": 'South'}, 4: {"phase": 'J6', "direction": 'South'},
#                   5: {"phase": 'J7', "direction": 'West'}, 6: {"phase": 'J7', "direction": 'West'}, 7: {"phase": 'J7', "direction": 'West'},
#                   8: {"phase": 'J4', "direction": 'West'}, 9: {"phase": 'J4', "direction": 'West'},
#                   10: {"phase": 'J5', "direction": 'North'}, 11: {"phase": 'J5', "direction": 'North'}, 12: {"phase": 'J5', "direction": 'North'},
#                   13: {"phase": 'J2', "direction": 'North'}, 14: {"phase": 'J2', "direction": 'North'},
#                   15: {"phase": 'J3', "direction": 'East'} ,16: {"phase": 'J3', "direction": 'East'} ,17:{"phase": 'J3', "direction": 'East'},
#                   18: {"phase": 'J8', "direction": 'East'}, 19: {"phase": 'J8', "direction": 'East'}}

OCC_BUS = 30
OCC_AUTO = 1.5



roadMapToTargetPhase_forInsetionPhase = {'N1E2': '5', 'J2E3': '6', 'C4E4': '7', 'C1E1': '8'}
# roadMapToTargetPhase_forInsetionPhase_nonGreenCase = {'N1E2': '9', 'J2E3': '10', 'C4E4': '11', 'C1E1': '12'}

class RSU:


    def __init__(self,ID,location,detectionRange):
        self.RSU_ID = ID
        self.location = location
        self.detectionRange = detectionRange ##DSRC or V2X detection range
        #self.CycleAccumulated = 0
        self.trafficSignalCurrentState = 0
        self.trafficSignalCurrentProgramID = 0
        self.trafficSignalCurrentPhaseIndex = 0

        #self.trafficSignalPlan = []
        self.cycleAccumulated = True

        self.phaseCommandBuffer = {}
        self.phaseElapsedTime = {}



        ## 補償相關參數
        self.IsUnderSignalCompensation = 0  #是否正在執行補償
        self.canDoSignalCompensation = 1 #補償開關
        # 優先控制啟動開關
        self.PREEMPTION_ACTIVATION = 1
        self.signalStatus = 0  # [0] = pretimed [1] = preemption [2] = compensating
        self.hasDonePreemption = 0

        ### 臨時用 不是正式變數 20210830 ###
        ## 為處理20210830問題 -> 緊急車不明原因沒有闖紅燈) ##
        self.preEmptionStrategy = 0

    ##################  Subfunctions ####################

    ## private function
    def __getPhaseCountOfTrafficSignalProgram(self, programID):
        count = 0
        logics = traci.trafficlight.getAllProgramLogics(self.RSU_ID)
        for logicObject in logics:
            if logicObject.programID == programID:
                count = len(logicObject.phases)
        if count == 0:
            print("exception: phaseCount = 0")
            print(1 / 0)
        return count

    def __constructPhaseCommandBuffer(self):

        programID = traci.trafficlight.getProgram(self.RSU_ID)
        phaseCount = self.__getPhaseCountOfTrafficSignalProgram(programID=programID)

        for phaseNum in range(phaseCount):
            self.phaseCommandBuffer.update({phaseNum: {'switch': 'No', 'extent': 0, 'truncation': 0, 'hasDone': False}})
        print(self.phaseCommandBuffer)

    def __constructPhaseElapsedTimeDICT(self):

        programID = traci.trafficlight.getProgram(self.RSU_ID)
        phaseCount = self.__getPhaseCountOfTrafficSignalProgram(programID=programID)
        for phaseNum in range(phaseCount):
            self.phaseElapsedTime.update({phaseNum: 0})
        print(self.phaseElapsedTime)


    def initilization(self):
        self.__constructPhaseElapsedTimeDICT()
        self.__constructPhaseCommandBuffer()

    def checkPhaseCommandBuffer(self):

        def checkMinDur(self, phaseID):

            def getTrafficSignalCurrentProgram(self):
                nowProgramID = traci.trafficlight.getProgram(self.RSU_ID)
                programs = traci.trafficlight.getAllProgramLogics(self.RSU_ID)
                for logicObject in programs:
                    if logicObject.programID == nowProgramID:
                        return logicObject

            #remainTime = traci.trafficlight.getNextSwitch(self.RSU_ID) - traci.simulation.getTime()
            #elapsedTime = traci.trafficlight.getPhaseDuration(self.RSU_ID) - remainTime
            currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)
            elapsedTime = self.phaseElapsedTime[currentPhaseID]
            program = getTrafficSignalCurrentProgram(self)
            minDur = program.phases[phaseID].minDur

            if elapsedTime >= minDur:
                return True
            else:
                return False

        if self.PREEMPTION_ACTIVATION == 1: #確認可執行優先
            currentPhaseID = traci.trafficlight.getPhase(self.RSU_ID)
            if not self.phaseCommandBuffer[currentPhaseID]['hasDone']:
                if self.phaseCommandBuffer[currentPhaseID]['switch'] == 'Yes':
                    hasCompeletedMinDur = checkMinDur(self, phaseID=currentPhaseID) #切斷時相: 要先看是否已滿足最小綠
                    if hasCompeletedMinDur:
                        traci.trafficlight.setPhaseDuration(tlsID=self.RSU_ID, phaseDuration=0) #切斷
                        self.phaseCommandBuffer[currentPhaseID]['hasDone'] = True
                        print("xxxx")
                elif self.phaseCommandBuffer[currentPhaseID]['switch'] == 'No':
                    phaseAdjustAmount = self.phaseCommandBuffer[currentPhaseID]['extent'] - self.phaseCommandBuffer[currentPhaseID]['truncation']
                    remainTime = traci.trafficlight.getNextSwitch(self.RSU_ID) - traci.simulation.getTime()
                    newPhaseDuration = remainTime + phaseAdjustAmount
                    if newPhaseDuration <= 0: #縮短秒數
                        hasCompeletedMinDur = checkMinDur(self, phaseID=currentPhaseID)
                        if hasCompeletedMinDur: #縮短秒數: 要先看是否已滿足最小綠
                            traci.trafficlight.setPhaseDuration(tlsID=self.RSU_ID, phaseDuration=newPhaseDuration)  # 縮短
                    else: #延長秒數
                        traci.trafficlight.setPhaseDuration(tlsID=self.RSU_ID, phaseDuration=newPhaseDuration)  # 延長
                    self.phaseCommandBuffer[currentPhaseID]['hasDone'] = True

                else: # 表示 self.phaseCommandBuffer[currentPhaseID]['switch'] == programID
                    #插入時相策略
                    traci.trafficlight.setProgram(tlsID=self.RSU_ID, programID=self.phaseCommandBuffer[currentPhaseID]['switch']) # 執行插入時相Program
                    traci.trafficlight.setPhase(tlsID=self.RSU_ID, index=2) #從第0個時相開始
                    self.cleanPhaseCommandBuffer()
                    #traci.trafficlight.setPhase(tlsID='I1', index=currentPhaseID)
                    self.phaseCommandBuffer[currentPhaseID]['hasDone'] = True
        else:
            print("不存取CommandBuffer: 優先控制啟動 = {0}".format(self.PREEMPTION_ACTIVATION))

        return 0

    def updateCommandToCommandBuffer(self, assignedPhaseID, commandContent):

        switch = commandContent['switch']
        extent = commandContent['extent']
        truncation = commandContent['truncation']

        self.phaseCommandBuffer[assignedPhaseID]['switch'] = switch
        self.phaseCommandBuffer[assignedPhaseID]['extent'] = extent
        self.phaseCommandBuffer[assignedPhaseID]['truncation'] = truncation
        self.phaseCommandBuffer[assignedPhaseID]['hasDone'] = False #新指令: 標記該指令尚未執行

    def executeSignalPreemption(self, obu, preemptionStrategy):
        from itertools import cycle
        from itertools import islice

        self.preEmptionStrategy = preemptionStrategy  ### 臨時用 不是正式變數 20210830 ### 為處理20210830問題 -> 緊急車不明原因沒有闖紅燈) ##

        def canPerformPreemption(self):
            #不做優先控制的條件 滿足下列任一就不做優先: (1) 已經執行過優先控制 (2) 號誌在補償 (3) 雲端(仿)是否有開啟優先號誌功能
            if self.hasDonePreemption or self.IsUnderSignalCompensation or (not self.PREEMPTION_ACTIVATION):
                return False
            else:
                return True

        if canPerformPreemption(self=self):
            currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)
            if preemptionStrategy == 1: #策略1: 綠燈延長，紅燈切斷
                if obu.nextTLSState in ['G', 'g'] and currentPhaseID not in [3, 9]: #前方是綠燈且不是左轉保護時相
                    command = {'switch': 'No', 'extent': 120, 'truncation': 0} ##延長綠燈120秒: 包裝指令
                    self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID, commandContent=command) #更新到commandBuffer
                else: #每個phase執行切斷時相
                    pool = cycle(self.phaseCommandBuffer)
                    start_at_next_phase = islice(pool, currentPhaseID, None)
                    for phaseID in start_at_next_phase:
                        if phaseID != obu.targetPhase:
                            command = {'switch': 'Yes', 'extent': 0, 'truncation': 0}  # 包裝指令: 切斷該時相
                            self.updateCommandToCommandBuffer(assignedPhaseID=phaseID,
                                                              commandContent=command)  # 更新到commandBuffer
                        else:
                            command = {'switch': 'No', 'extent': 120, 'truncation': 0}  # 包裝指令: 目標時相延長120秒
                            self.updateCommandToCommandBuffer(assignedPhaseID=phaseID,
                                                              commandContent=command)  # 更新到commandBuffer
                            break
            elif preemptionStrategy == 2: #策略2: 插入時相 - 待分相結束後再插入
                if obu.nextTLSState in ['G', 'g']: #是綠燈 -> 直接插入
                    obuOnRoadID = traci.vehicle.getRoadID(vehID=obu.OBU_ID)
                    programID = roadMapToTargetPhase_forInsetionPhase[obuOnRoadID]
                    command = {'switch': programID, 'extent': 0, 'truncation': 0}  # 包裝指令: 插入時相
                    self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID,
                                                      commandContent=command)  # 更新到commandBuffer
                else: #不是綠燈 -> 待紅燈結束後再插入
                    currentProgramPhaseList = self.getCurrentProgramContent()
                    nextPhaseID = currentPhaseID + 1
                    pool = cycle(currentProgramPhaseList)
                    loop_start_at_next_phase = islice(pool, nextPhaseID, None)  # 從下一個時相開始迴圈
                    for phase in loop_start_at_next_phase:
                        if phase.state == 'rrrrrrrrrrrrrrrrrrrr': #遇到全紅時相
                            obuOnRoadID = traci.vehicle.getRoadID(vehID=obu.OBU_ID)
                            programID = roadMapToTargetPhase_forInsetionPhase[obuOnRoadID]
                            command = {'switch': programID, 'extent': 0, 'truncation': 0}  # 包裝指令: 插入時相
                            self.updateCommandToCommandBuffer(assignedPhaseID=nextPhaseID, commandContent=command)
                            break
                        else:
                            print("非全紅時相")
                        nextPhaseID = nextPhaseID + 1
            elif preemptionStrategy == 3: #策略3: 插入時相 - 直接切斷
                # 條件1: 檢查公車目標時相目前是否為綠燈
                if obu.nextTLSState in ['G', 'g']: #是綠燈
                    obuOnRoadID = traci.vehicle.getRoadID(vehID=obu.OBU_ID)
                    programID = roadMapToTargetPhase_forInsetionPhase[obuOnRoadID]
                    command = {'switch': programID, 'extent': 0, 'truncation': 0}  # 包裝指令: 2 插入時相
                    self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID, commandContent=command)  # 更新到commandBuffer
                else: #不是綠燈
                    #條件2: 檢查currentPhase是否為綠燈 (競向是否正在執行綠燈)
                    currentPhaseState = traci.trafficlight.getAllProgramLogics(tlsID=self.RSU_ID)[4].phases[currentPhaseID].state
                    if 'g' in currentPhaseState or 'G' in currentPhaseState: #Yes: current phase is green
                        command =  {'switch': 'Yes', 'extent': 0, 'truncation': 0} #1.先塞切斷當下時相
                        self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID, commandContent=command)
                        self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID+1, commandContent=command) #currentPhaseID+1 -> 黃燈時相
                        obuOnRoadID = traci.vehicle.getRoadID(vehID=obu.OBU_ID)
                        programID = roadMapToTargetPhase_forInsetionPhase[obuOnRoadID]
                        command = {'switch': programID, 'extent': 0, 'truncation': 0}  # 2.再插入時相
                        self.updateCommandToCommandBuffer(assignedPhaseID=currentPhaseID+2, commandContent=command) #currentPhaseID+2 -> 全紅時相
                    else: #現在不是綠燈
                        pool = cycle(self.phaseCommandBuffer)
                        loop_start_at_current_phase = islice(pool, currentPhaseID, None)  # 從當下執行時相開始迴圈
                        for phaseID in loop_start_at_current_phase:
                            #nowProgramID = traci.trafficlight.getProgram(tlsID=self.RSU_ID)
                            phaseState = traci.trafficlight.getAllProgramLogics(tlsID=self.RSU_ID)[4].phases[phaseID].state #[4]為定時時制
                            if 'g' in phaseState or 'G' in phaseState: #若遇到有綠燈的時相
                                obuOnRoadID = traci.vehicle.getRoadID(vehID=obu.OBU_ID)
                                programID = roadMapToTargetPhase_forInsetionPhase[obuOnRoadID]
                                command = {'switch': programID, 'extent': 0, 'truncation': 0}  # 包裝指令: 插入時相
                                # 將插入時相放到前一個時相中
                                if phaseID != 0:
                                    self.updateCommandToCommandBuffer(assignedPhaseID=phaseID-1,
                                                                      commandContent=command)  # 更新到commandBuffer
                                else:
                                    self.updateCommandToCommandBuffer(assignedPhaseID=11,
                                                                      commandContent=command)  # 更新到commandBuffer
                                break
                            else:
                                command = {'switch': 'Yes', 'extent': 0, 'truncation': 0}  # 包裝指令: 切斷該時相
                                self.updateCommandToCommandBuffer(assignedPhaseID=phaseID,
                                                                  commandContent=command)  # 更新到commandBuffer
            elif preemptionStrategy == 0:
                print("ddd")

            self.canDoSignalCompensation = 0 #執行號誌優先時不可執行補償
        else:
            print("不啟動優先控制: 優先控制啟動 = {0}".format(self.PREEMPTION_ACTIVATION))

        self.hasDonePreemption = 1 #標記已做過優先控制，下一秒進來不可以再做優先控制
        return 0

    def cleanPhaseCommandBuffer(self):
        for phaseID in self.phaseCommandBuffer:
            currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)

            # if phaseID == currentPhaseID: #當下執行的phase直接切斷
            #     command = {'switch': 'Yes', 'extent': 0, 'truncation': 0}
            #     self.updateCommandToCommandBuffer(assignedPhaseID=phaseID, commandContent=command)
            # else: #其他phase的command 清除
            #     command = {'switch': 'No', 'extent': 0, 'truncation': 0}
            #     self.updateCommandToCommandBuffer(assignedPhaseID=phaseID, commandContent=command)

            command = {'switch': 'No', 'extent': 0, 'truncation': 0}
            self.updateCommandToCommandBuffer(assignedPhaseID=phaseID, commandContent=command)

    def executeSignalCompensation(self):

        def canExecuteSignalCompensation(self):

            def checkCurrentProgramIsFixedTime(self):
                if self.getCurrentProgramID() == '99':
                    return True
                else: return False
            #執行補償條件: (1)號誌為定時制  (2)號誌設定允許執行補償
            if checkCurrentProgramIsFixedTime(self=self) and (self.canDoSignalCompensation) :
                return True
            else: return False

        def getCurrentStepOfCycle(self):
            #取得現在是cycle的第幾秒
            currentPhaseID = self.getCurrentPhaseID()
            currentPhaseEndTime = self.getCurrentPhaseEndTime()
            phaseList = self.getCurrentProgramContent()
            phaseID = 0
            step = 0
            for phase in phaseList:
                if currentPhaseID == phaseID:
                    if currentPhaseEndTime == traci.simulation.getTime(): #這個phase已經走到底
                        step = step + self.getFixedTimePhaseDuration(programID=self.trafficSignalCurrentProgramID,
                                                                     phaseID=currentPhaseID)
                    else:
                        step = step + (currentPhaseEndTime - traci.simulation.getTime())
                    break
                else:
                    step = step + self.phaseElapsedTime[phaseID]
                phaseID = phaseID + 1

            return step

        def getSupposedStepOfCycle(self):
            #取得原預計是cycle的第幾秒
            nowSimulationTime = traci.simulation.getTime()
            currentProgramID = self.getCurrentProgramID()
            cycleLength = self.getAssignedProgramCycleLength(programID=currentProgramID)
            step = nowSimulationTime % cycleLength
            if step == 0:
                step = cycleLength
            return step

        # 補償機制
        if canExecuteSignalCompensation(self=self):
            #supposedStep = getSupposedStepOfCycle(self=self)
            #currentStep = getCurrentStepOfCycle(self=self)
            currentProgramID = self.getCurrentProgramID()
            cycleLength = self.getAssignedProgramCycleLength(programID=currentProgramID)
            if ((traci.simulation.getTime() + 1) % cycleLength == 0): #phase 11 + 1秒  = 週期秒數 可被整除表示不用補償
                self.IsUnderSignalCompensation = 0  # 標記沒有執行補償
                return 0
            else:
                diff = (traci.simulation.getTime() + 1) % cycleLength
                if (diff <= round(cycleLength/2)): adjusmentAmountOfFirstCycle = -(diff)
                else: adjusmentAmountOfFirstCycle = (cycleLength - diff)

            #adjusmentAmountOfSecondCycle = math.floor(diff / 2)

            # if diff == 0:
            #     self.IsUnderSignalCompensation = 0 #標記沒有執行補償
            #     return 0

            cycleLength = self.getAssignedProgramCycleLength(programID=self.getCurrentProgramID())
            phaseList = self.getCurrentProgramContent()
            phaseID = 0
            for phase in phaseList:
                if ('G' in phase.state) or ('g' in phase.state):
                    result = adjusmentAmountOfFirstCycle * (phase.duration + 5) / cycleLength # +5 為黃燈&全紅比例，暫時這樣寫，正確應該讀取兩者數值
                    if result > 0:
                        adjustAmountOfPhase = math.ceil(result)
                    else: adjustAmountOfPhase = math.floor(result)
                    if adjustAmountOfPhase > 0:
                        command = {'switch': 'No', 'extent': adjustAmountOfPhase, 'truncation': 0}  # 包裝指令
                    else:
                        command = {'switch': 'No', 'extent': 0, 'truncation': abs(adjustAmountOfPhase)}  # 包裝指令
                    self.updateCommandToCommandBuffer(assignedPhaseID=phaseID, commandContent=command)
                    phase.duration

                phaseID = phaseID + 1

            print("dddd")
            #self.IsUnderSignalCompensation = 1 #表示正在補償狀態 ## 20210826 暫時拿掉 避免問題

    def resumeToFixedTimePlan(self):

        nowProgramID = traci.trafficlight.getProgram(tlsID=self.RSU_ID)
        currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)

        if nowProgramID != '99':  # 若正在執行插入時相program -> 切換回原定時制program (0)
            command = {'switch': 'Yes', 'extent': 0, 'truncation': 0}  # 包裝指令: 切斷分相
            self.updateCommandToCommandBuffer(assignedPhaseID=2, commandContent=command)
            self.updateCommandToCommandBuffer(assignedPhaseID=3, commandContent=command)
            command = {'switch': 99, 'extent': 0, 'truncation': 0}  # 包裝指令: 切換至定時時制
            self.updateCommandToCommandBuffer(assignedPhaseID=4, commandContent=command)

            for item in self.phaseElapsedTime:
                self.phaseElapsedTime[item] = 0  # 將phaseElapsedTime歸0 避免重複計數

        else:  # 正在執行定時制
            currentPhaseElapsedTime = self.phaseElapsedTime[currentPhaseID] #取得時相已經過時間
            currentPhaseFixedTimeDuration = self.getFixedTimePhaseDuration(programID=nowProgramID, phaseID=currentPhaseID) #取得時相定時制時間長度
            newDuration = max(0, currentPhaseFixedTimeDuration - currentPhaseElapsedTime) #比較已經經過時間與定時制長度，若已經過>定時制長度 -> 直接切斷，否則更新剩餘秒數為剩下多少秒
            traci.trafficlight.setPhaseDuration(tlsID=self.RSU_ID, phaseDuration=newDuration) #設定新時相長度
            self.cleanPhaseCommandBuffer()  # 清除commandBuffer

    def cancelSignalPreemption(self):

        if self.PREEMPTION_ACTIVATION != 0:
            self.resumeToFixedTimePlan() #回復到定時制
            self.hasDonePreemption = 0 #恢復可接受優先控制
            self.canDoSignalCompensation = 1 #恢復可執行補償

            self.preEmptionStrategy = 0
        else:
            print("不須回復號誌: 優先控制啟動 = {0}".format(self.PREEMPTION_ACTIVATION))

        return 0

    def updateTrafficSignalInformation(self):

        # 更新phaseElapsedTime
        def updatePhaseElapsedTime(self):
            currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)
            programID = traci.trafficlight.getProgram(self.RSU_ID)
            phaseCount = self.__getPhaseCountOfTrafficSignalProgram(programID=programID)
            lastPhase = phaseCount - 1
            if currentPhaseID == 0 and self.phaseElapsedTime[lastPhase] != 0:
                for phase in self.phaseElapsedTime:
                    self.phaseElapsedTime[phase] = 0  # 已經走完一遍週期: 將elapsedTime全部清除歸0
            self.phaseElapsedTime[currentPhaseID] = self.phaseElapsedTime[currentPhaseID] + 1

        self.trafficSignalCurrentState = traci.trafficlight.getRedYellowGreenState(tlsID=self.RSU_ID)
        self.trafficSignalCurrentProgramID = traci.trafficlight.getProgram(tlsID=self.RSU_ID)
        self.trafficSignalCurrentPhaseIndex = traci.trafficlight.getPhase(tlsID=self.RSU_ID)
        updatePhaseElapsedTime(self=self)

        # 統一在phase 11的第一秒執行補償檢查
        if self.trafficSignalCurrentPhaseIndex == 11 and (self.getCurrentPhaseEndTime() - traci.simulation.getTime() == 1):
            self.executeSignalCompensation()
        #Time out檢查 -> 在策略1 優先號誌狀態下，時相已執行秒數 > 門檻值(35秒)即取消優先號誌 (為處理20210830問題 -> 緊急車不明原因沒有闖紅燈)
        currentPhaseID = traci.trafficlight.getPhase(tlsID=self.RSU_ID)
        if self.phaseElapsedTime[currentPhaseID] > 35 and self.preEmptionStrategy == 1:
            traci.trafficlight.setPhaseDuration(tlsID=self.RSU_ID, phaseDuration=0) #切斷


    ################################################################

    def addTrafficSignalPlan(self, plan):
        self.trafficSignalPlan.append(plan)

    def setRSU_ID(self,ID):
        self.RSU_ID = ID

    def setLoaction(self,location):
        self.location = location

    def getCurrentPhaseState(self):
        self.trafficSignalcurrentState = traci.trafficlight.getRedYellowGreenState(tlsID=self.RSU_ID)
        return traci.trafficlight.getRedYellowGreenState(tlsID=self.RSU_ID)

    def getCurrentPhaseID(self):
        return traci.trafficlight.getPhase(tlsID=self.RSU_ID)

    def getCurrentPhaseEndTime(self):
        return traci.trafficlight.getNextSwitch(tlsID=self.RSU_ID)

    def getCurrentProgramContent(self):
        currentProgramID = self.getCurrentProgramID()
        allProgramLogics = traci.trafficlight.getAllProgramLogics(tlsID=self.RSU_ID)
        try:
            for program in allProgramLogics:
                if program.programID == currentProgramID:
                    phasesList = program.phases
        except:
            raise Exception("找不到指定的program")
        return phasesList

    def getCurrentProgramID(self):
        return traci.trafficlight.getProgram(tlsID=self.RSU_ID)

    def getAssignedProgramContent(self, programID):
        allProgramLogics = traci.trafficlight.getAllProgramLogics(tlsID=self.RSU_ID)
        try:
            for program in allProgramLogics:
                if program.programID == programID:
                    phasesList = program.phases
        except:
            raise Exception("找不到指定的program")
        return phasesList

    def getAssignedProgramCycleLength(self, programID):
        phaseList = self.getAssignedProgramContent(programID=programID)
        cycle = 0
        for phase in phaseList:
            cycle = cycle + phase.duration
        return cycle


    def getFixedTimePhaseDuration(self, programID, phaseID):
        phasesList = self.getAssignedProgramContent(programID=programID)
        return phasesList[phaseID].duration

    def getVehicleParameters(self, vehID):

        def getVehOccupancy(vehType):
            if vehType == "Bus":  # 設定乘載人數
                occupancy = OCC_BUS
            elif vehType == "Special":
                occupancy = ALPHA * OCC_AUTO
            else:
                occupancy = OCC_AUTO
            return occupancy


        try:
            nextTLSID = traci.vehicle.getNextTLS(vehID)[0][0]  # String: 'I1', 'I2', 'I3'....
            nextTLSIndex = traci.vehicle.getNextTLS(vehID)[0][1]  # Int: ex. 0~19
            dist = traci.vehicle.getNextTLS(vehID)[0][2]  # distance
            nextTLSState = traci.vehicle.getNextTLS(vehID)[0][3] #state ex. 'G' 'y' 'r'

        except IndexError as error:
            # print("IndexError: ",error)
            nextTLSID = None
            nextTLSIndex = 99999
            dist = 99999
            nextTLSState = 99999

        vehType = traci.vehicle.getTypeID(vehID)

        if (nextTLSID == self.RSU_ID and dist <= self.detectionRange): # 確認車輛前方路口屬於正確RSU
            # 兩條件可以加入計算: 車輛離路口距離小於DSRC偵測範圍
            activation = 'yes'
            #TPxj = dist / Vf + traci.simulation.getTime() - self.CycleAccumulated + 2
            #phase = TLS_index_pair[nextTLSIndex]['phase']
            #direction =  TLS_index_pair[nextTLSIndex]['direction']
        else:
            activation = 'no'
            # TPxj = None
            # phase = None
            # direction = 'None'

        #order = 9999  # 暫時給予，在 sortQueueList 中再進行編號
        vehSpeed = traci.vehicle.getSpeed(vehID)
        position = traci.vehicle.getPosition(vehID)
        occupancy = getVehOccupancy(vehType=vehType)


        vehPara = {"vehID": vehID, "Activation": activation, "type": vehType, "occupancy": occupancy,
                   "vehSpeed": vehSpeed, "nextTLSID": nextTLSID, "nextTLSIndex": nextTLSIndex,
                   "nextTLSState": nextTLSState, "dist": dist, "position": position}

        return vehPara

    def checkVehExisted(self, vehID):
        vehIDList = traci.vehicle.getIDList()
        if vehID in vehIDList:
            return True
        else:
            return False

    def __str__(self):
        return 'RSU(ID = {0}, location = {1},  plan = {2})'\
            .format(self.RSU_ID, self.location, self.plan)





