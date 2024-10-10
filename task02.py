from app.drone.Drone import Drone
from floripa.Camera import main as measure_form
from app.drone.tools.GPS import GPS

drone = Drone()
GPS = GPS()

# parser = argparse.ArgumentParser(description='...')
# parser.add_argument('--target', help="lat and long of land")
# args = parser.parse_args()

# home = args.home
# home = home.split(",")
# lat, lon = map(float, home)

lat, lon, alt = drone.get_gps_position()



if not drone.connected():
    exit()

drone.disable_pre_arm_checks()
drone.change_to_guided_mode()
drone.set_home(lat, lon)

drone.arm_drone()

drone.ascend(1.5)
# drone.move_forward(20)
# --home=33.2110977,-87.5367255,0,0

new_lat, new_lon = GPS.meters_to_geo(lat, lon, 3, 90)
drone.move_to_position(new_lat, new_lon)
measure_form(drone.current_altitude())
drone.return_to_home()

# drone.land()
# drone.disarm()


