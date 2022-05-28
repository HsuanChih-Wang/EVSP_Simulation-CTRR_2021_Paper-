import os
import sys
import configparser

from multiprocessing import Pool

if __name__ == "__main__":

    VC = ['1', '2']
    EV_Level = ['1', '2', '3']
    LeftLevel = ['1', '3']
    ControlStrategy = ['0']
    NumOfRuns = 5

    # Config File Parser
    config = configparser.ConfigParser()
    config.read('Config.ini')
    # config.read('Config_forVC0.5.ini')
    # if VC == '1':
    #     config.read('Config_forVC0.5.ini')
    # elif VC == '2':
    #     config.read('Config_forVC0.9.ini')

    def executeSUMO():
        os.system("start1.bat")

    for vc in VC:
        for ev_level in EV_Level:
            for left_level in LeftLevel:
                for controlstrategy in ControlStrategy:
                    config['DEFAULT']['VC'] = vc
                    config['DEFAULT']['EV_Level'] = ev_level
                    config['DEFAULT']['LeftLevel'] = left_level
                    config['DEFAULT']['ControlStrategy'] = controlstrategy
                    with open('Config.ini', 'w') as configfile:    # save
                        config.write(configfile)
                    for runs in range(NumOfRuns):
                        executeSUMO()

    # def test_run(pool):
    #     for run in range(NumOfRuns):
    #         pool.apply_async(os.system("start1.bat"))
    #
    # pool = mp.Pool()
    # test_run(pool=pool)
    # pool.close()
    # pool.join()

    # with Pool(4) as pool:
    #     for runs in range(NumOfRuns):
    #         pool.apply_async(executeSUMO())
    #     pool.close()
    #     pool.join()





