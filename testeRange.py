from app.drone.Drone import Drone
import math

drone = Drone()

if not drone.connected():
    print("Drone not connected")
    exit()

print(math.trunc(drone.get_rangefinder_distance() * 100))
