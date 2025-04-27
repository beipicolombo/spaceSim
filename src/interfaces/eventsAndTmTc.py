
import numpy as np

class Telecommand():
    def __init__(self, tcName = ""):
        self.name = tcName
        self.id = 0
        self.time = 0

class Event():
    def __init__(self, name = "NULL", id = 0, time = 0):
        self.name = name
        self.id = id
        self.time = time

    def display(self):
        print("EVENT at elapsed time: " + str(self.time) + " [s] " + self.name + " ID: " + str(self.id))