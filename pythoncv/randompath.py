import random
import asyncio
import time

class botPath:

    def __init__(self, cart):
        self.bot = cart
        self.maxRPM = 10
        self.goTime = 0

    async def moveForward(self, client):
        wheelspeed = random.randint(1, self.maxRPM)
        print(f"{-wheelspeed}, {wheelspeed}")
        await self.bot.set_wheel_speed(-wheelspeed, wheelspeed, client)
        # time.sleep(random.random())

    async def turn(self, client):
        wheelspeed = random.randint(1, self.maxRPM)

        await self.bot.set_wheel_speed(wheelspeed, wheelspeed, client)
        # time.sleep(random.random())

    async def moveBack(self, client):
        print(f"{self.maxRPM}, {-self.maxRPM}")
        await self.bot.set_wheel_speed(self.maxRPM, -self.maxRPM, client)

    async def takePath(self, client, coords):
        '''
        bottom left: 0, 0
        top left: 0, 50
        top right: 86, 52
        bottom right: 86, 0
        '''
        print(client.is_connected)
        while True:
            if coords.xCoor > 70 or coords.xCoor < 10 or coords.yCoor < 10 or coords.yCoor > 40:
                print("moving back")
                await self.moveBack(client)
                self.goTime = 0
            elif self.goTime == 30:
                choice = random.randint(1, 3)
                if choice < 3:
                    print("forward")
                    await self.moveForward(client)
                else:
                    print("turn")
                    await self.turn(client)
                self.goTime = 0

            self.goTime += 1
            print("done")
            await asyncio.sleep(1 / 30)




