import os, time
from QcarFleet import QcarFleet
from ControlLeader import ControlLeader
from ControlFollower import ControlFollower

def main():
    SimTime = 40
    LeaderIndex = 0
    QcarNum = 5
    DistanceBetweenEachCar          = 0.2
    Controller                      = "CACC"
    Observer                        = ""

    Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer)
    print("Create Control Threading")
    LeaderControl = ControlLeader(SimTime, False, [0,1], True)
    print("Leader Control Start")
    
    FollowerControl = []
    for i in range(1,QcarNum):
        FollowerControl.append(ControlFollower(SimTime, Fleet, i, i-1))

    for i in range(0,QcarNum-1):
        FollowerControl[i].start()
        time.sleep(0.1)
    LeaderControl.start()
    
    for i in range(0,QcarNum-1):
        FollowerControl[i].join()
        time.sleep(0.1)
    LeaderControl.join()

    try:
        print("Main Thread running")
        while LeaderControl.is_alive() or FollowerControl[QcarNum-1].is_alive():

            time.sleep(0.1)
    except:
        LeaderControl.stop()
        print("Thread Stop")
    finally:
        print("Simulation Ends.")
        # input('Experiment complete. Press any key to exit...')

if __name__ == "__main__":
    main()