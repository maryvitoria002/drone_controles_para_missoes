from app.drone.Drone import Drone
from floripa.Camera import main as measure_form
from app.drone.tools.GPS import GPS

drone = Drone()
GPS = GPS()

lat, lon, alt = drone.get_gps_position()

if not drone.connected():
    exit()

drone.disable_pre_arm_checks()
drone.change_to_guided_mode()
drone.set_home(lat, lon)

drone.arm_drone()

drone.ascend(1.5)

new_lat, new_lon = GPS.meters_to_geo(lat, lon, 3, 90)
drone.move_to_position(new_lat, new_lon)
measure_form(drone.current_altitude())
drone.return_to_home()




