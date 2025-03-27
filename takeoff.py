from app.drone.Drone import Drone
import time

drone = Drone()

if not drone.connected():
    print("Drone not connected")
    exit()

def run():
    try:
        lat, lon, _ = drone.get_gps_position()
        drone.set_home(lat, lon)
        drone.change_to_guided_mode()
        drone.arm_drone()
        drone.takeoff(6)
        time.sleep(1)
        drone.go_to_location(0, 0, 3)
        time.sleep(3)
        drone.land()
        drone.disarm()

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
