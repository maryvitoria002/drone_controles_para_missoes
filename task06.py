from app.drone.Drone import Drone
from app.drone.tools.GPS import GPS
import argparse

drone = Drone()
gps = GPS()

lat, lon, alt = drone.get_gps_position()

if not drone.connected():
    exit()

def run():
    try:
        drone.change_to_guided_mode()
        drone.set_home(lat, lon)
        new_lat, new_lon = gps.meters_to_geo(lat, lon, 6, 180)

        drone.arm_drone()
        drone.ascend(2.5)
        drone.move_to_position(new_lat, new_lon)
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

def run2():
    try:
        drone.change_to_guided_mode()
        drone.set_home(lat, lon)

        drone.arm_drone()
        drone.ascend(2.5)
        drone.move_to_position(coord_b[0], coord_b[1])
        drone.land()
        drone.disarm()

        check = wait_to_continue()
        
        if check == True:
            drone.arm_drone()
            drone.ascend(2.5)
            drone.return_to_home()
        else:
            exit()

    except KeyboardInterrupt:
        drone.land()
        drone.disarm()
        exit()

    except Exception as e:
        print(e)
        drone.land()
        drone.disarm()
        exit()

def wait_to_continue():
    while True:
        msg = drone.conn.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=True)
        if msg.chan6_raw:
            if msg.chan6_raw <= 990:
                return True
run()



