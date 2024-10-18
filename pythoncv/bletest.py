import random
import robotClassBluePy
import time

bot = robotClassBluePy.INRbot("a6:5d:28:70:b8:e2", ["bac3","78d3","2bef"])
# device address c8:09:a8:94:92:69

while True:    #bot.s
    start = time.perf_counter_ns()
    success, data = bot.receiveNotification(1/30)
    if success:
        end = time.perf_counter_ns()
        print((end-start)/(1e6))
        print(data)
    else:
        print("No data!")
