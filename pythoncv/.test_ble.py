import random
import robotClassBluePy
import time

bot = robotClassBluePy.INRbot("ec:62:60:8e:60:16", ["bac3","78d3","2bef"])
# device address c8:09:a8:94:92:69

while True:    #bot.s
    start = time.perf_counter_ns()
    success, data = bot.receiveNotification(1/30)
    bot.setWheelSpeed(int(10*random.random()), int(10*random.random()))
    if success:
        end = time.perf_counter_ns()
        print((end-start)/(1e6))
        #print(data)
    else:
        print("No data!")


