import random

class botPath:

    def __init__(self, cart):
        self.bot = cart
        self.maxRPM = 50

    async def moveForward(self, client):
        wheelspeed = random.randint(1, self.maxRPM)
        await self.bot.set_wheel_speed(wheelspeed, -wheelspeed, client)
        # time.sleep(random.random())

    async def turn(self, client):
        leftSpeed = random.randint(-self.maxRPM, self.maxRPM)
        rightSpeed = random.randint(-self.maxRPM, self.maxRPM)
        await self.bot.set_wheel_speed(leftSpeed, rightSpeed, client)
        # time.sleep(random.random())

    async def takePath(self, client):
        choice = random.randint(1, 3)
        #print(choice)
        if choice < 3:
            #print("forward")
            await self.moveForward(client)
        else:
            #print("turn")
            await self.turn(client)

