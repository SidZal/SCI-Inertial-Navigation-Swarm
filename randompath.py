import random

class botPath:

    def __init__(self, bart):
        self.bot = bart
        self.maxRPM = 50

    def moveForward(self):
        wheelspeed = random.randint(1, self.maxRPM)
        self.bot.setWheelSpeed(wheelspeed, wheelspeed)
        # time.sleep(random.random())

    def turn(self):
        leftSpeed = random.randint(-self.maxRPM, self.maxRPM)
        rightSpeed = random.randint(-self.maxRPM, self.maxRPM)
        self.bot.setWheelSpeed(leftSpeed, rightSpeed)
        # time.sleep(random.random())

    def takePath(self):
        choice = random.randint(1, 2)
        if choice == 1:
            self.moveForward()
        else:
            self.turn()

