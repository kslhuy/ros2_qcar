import os, time
from QcarFleet import QcarFleet
from ControllerLeader import LeaderControl

def main():
    LeaderIndex = 0
    QcarNum = 5
    DistanceBetweenEachCar          = 0.2
    Controller                      = "CACC"
    Observer                        = ""

    Fleet = QcarFleet(QcarNum, LeaderIndex, DistanceBetweenEachCar, Controller, Observer)
    print("Create Control Threading")
    Control = LeaderControl()
    print("LeaderControl Start")
    Control.start()
    try:
        while True:
            time.sleep(0.1)
    except:
        Control.stop()
        print("Thread Stop")
    finally:
        input('Experiment complete. Press any key to exit...')

if __name__ == "__main__":
    main()