from app.drone.Drone import Drone
from app.drone.tools.GPS import GPS
import time

drone = Drone()
lat, lon, alt = drone.get_gps_position()
gps = GPS()

if not drone.connected():
    print("Drone not connected")
    exit()

def run():
    try:
        lat, lon, _ = drone.get_gps_position()
        drone.set_home(lat, lon)
        new_lat, new_lon = gps.meters_to_geo(lat, lon, 0, 0)
        drone.change_to_guided_mode()
        drone.arm_drone()
        drone.takeoff(6)
        time.sleep(1)
        drone.move_to_position(new_lat, new_lon)
        time.sleep(1)
        drone.descend(3)
        # time.sleep(3)
        # drone.ascend(6)
        # time.sleep(1)
        # drone.land()
        # drone.disarm()

    except KeyboardInterrupt:
        drone.land()
        drone.disarm()
        exit()

    except Exception as e:
        print(e)
        drone.land()
        drone.disarm()
        exit()

run()
