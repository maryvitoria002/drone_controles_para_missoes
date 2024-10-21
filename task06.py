from app.drone.Drone import Drone
from app.drone.tools.GPS import GPS
import argparse

parser = argparse.ArgumentParser(description='...')
parser.add_argument('--coordb', help="lat and long of B point")
parser.add_argument('--distance', help="distance to point B")
parser.add_argument('--angle', help="angle to point B")

args = parser.parse_args()

coord_b = args.coordb
distance = int(args.distance)
angle = int(args.angle)

coord_b = coord_b.split(",")
coord_b = list(map(float, coord_b))

drone = Drone()
gps = GPS()

lat, lon, alt = drone.get_gps_position()

if not drone.connected():
    exit()

def run():
    try:
        drone.change_to_guided_mode()
        drone.set_home(lat, lon)
        new_lat, new_lon = gps.meters_to_geo(lat, lon, distance if distance else 2, angle if angle else 0)

        drone.arm_drone()
        drone.ascend(2.5)
        drone.move_to_position(new_lat, new_lon)
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

while True:
    msg = drone.conn.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=True)
    if msg.chan6_raw:
        if msg.chan6_raw >= 2014:
            run()
            break


