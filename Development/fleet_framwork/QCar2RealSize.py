from pal.products.qcar import QCar

class QCar2RealSize(QCar):
    """
    Real Size Qcar for other simulation environment.
    """
    def __init__(self, readMode=0, frequency=500, pwmLimit=0.9, steeringBias=0, hilPort=18960):
        super().__init__(readMode, frequency, pwmLimit, steeringBias, hilPort)

