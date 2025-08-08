import numpy as np

class Driver:

    speed = 0
    lane = 0
    angry = 0

    def __init__(self, max_lanes) -> None:

        self.max_lanes = max_lanes

    def honk(self):
        pass

    def lights(self):
        pass
    
    def switch_lane(self, direction:str):
        if direction == "left":
            if self.lane < 0:
                self.lane = 0 
            else:
                self.lane -= 1

        if direction == "right":
            if self.lane > self.max_lanes:
                self.lane = self.max_lanes  
            else:
                self.lane += 1

class Environment:

    cars = []
    driver = None

    def __init__(self) -> None:
        self.max_lanes = 4
        self.driver = Driver(self.max_lanes)
        pass

    def spawn_cars(self, lanes):

        for lane in lanes:

            car = Car(lane)

            self.cars.append(car)



class Car:
    lane = 0
    speed = 1
    position = 100

    def __init__(self, lane) -> None:
        self.lane = lane

    def update_position(self):
        
        self.position -= self.speed

        return self.position