from pal.products.qcar import QCar

class QCar2RealSize(QCar):
    def __init__(self, readMode=0, frequency=500, pwmLimit=3, steeringBias=0, hilPort=18960):
        super().__init__(readMode, frequency, pwmLimit, steeringBias, hilPort)

