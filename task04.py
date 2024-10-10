from app.drone.Drone import Drone
import argparse
import time 

parser = argparse.ArgumentParser(description='...')
parser.add_argument('--coorda', help="lat and long of A point")
parser.add_argument('--coordb', help="lat and long od B point")
args = parser.parse_args()


coord_a = args.coorda
coord_b = args.coordb

if not coord_a or not coord_b:
    exit()

coord_a = coord_a.split(",")
coord_b = coord_b.split(",")

coord_a = list(map(float, coord_a))
coord_b = list(map(float, coord_b))

drone = Drone()

if not drone.connected():
    exit()

lat, lon, alt = drone.get_gps_position()    

drone.disable_pre_arm_checks()
drone.change_to_guided_mode()

drone.set_home(coord_a[0], coord_a[1])

drone.ascend(3)

drone.move_to_position(coord_a[0], coord_a[1])
time.sleep(10)

drone.move_to_position(coord_b[0], coord_b[1])
time.sleep(10)

drone.return_to_home()
