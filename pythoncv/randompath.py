import random
import asyncio
import time

class botPath:

    def __init__(self, cart):
        self.bot = cart
        self.maxRPM = 50
        self.goTime = 0

    async def moveForward(self, client):
        wheelspeed = random.randint(1, self.maxRPM)
        await self.bot.set_wheel_speed(wheelspeed, -wheelspeed, client)
        # time.sleep(random.random())

    async def turn(self, client):
        wheelspeed = random.randint(1, self.maxRPM)
        await self.bot.set_wheel_speed(wheelspeed, wheelspeed, client)
        # time.sleep(random.random())

    async def takePath(self, client):
        while True:
            if self.goTime == 30:
                choice = random.randint(1, 3)
                if choice < 3:
                    print("forward")
                    await self.moveForward(client)
                else:
                    print("turn")
                    await self.turn(client)
                self.goTime = 0

            self.goTime += 1
            print(self.goTime)

            await asyncio.sleep(1 / 30)


    # async def takePath(self, client):
    #         #if time.perf_counter() % 3 == 0:
    #     choice = random.randint(1, 3)
    #             #print(choice)
    #     if choice < 3:
    #         print("forward")
    #         await self.moveForward(client)
    #     else:
    #         print("turn")
    #         await self.turn(client)
    #
    #     await asyncio.sleep(1 / 30)

        # if not client.is_connected:
        #     self.bot.set_wheel_speed(0, 0)



