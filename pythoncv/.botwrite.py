import csv

class BotWriter:
    
    def __init__(self, aruco_id):
        self.BOT_ID = aruco_id
        self.filename = aruco_id + ".csv"
        with open(self.filename, "a") as csvfile:
            self.write = csv.writer(csvfile)
            
            
    def writeRow(self, line):
        self.write.writerow(line)
        