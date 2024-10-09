from app.drone.Drone import Drone
from floripa.Camera import main as measure_form
import math
import argparse

drone = Drone()

# parser = argparse.ArgumentParser(description='...')
# parser.add_argument('--target', help="lat and long of land")
# args = parser.parse_args()

# home = args.home
# home = home.split(",")
# lat, lon = map(float, home)

lat, lon, alt = drone.get_gps_position()

def meters_to_geo(lat, lon, distance_m, bearing_deg):
    # Raio médio da Terra em metros
    R = 6371000  

    # Converter graus para radianos
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing_deg)

    # Calcular a nova latitude
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(distance_m / R) +
                         math.cos(lat_rad) * math.sin(distance_m / R) * math.cos(bearing_rad))

    # Calcular a nova longitude
    lon2_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance_m / R) * math.cos(lat_rad),
                                    math.cos(distance_m / R) - math.sin(lat_rad) * math.sin(lat2_rad))

    # Converter de volta para graus
    lat2 = math.degrees(lat2_rad)
    lon2 = math.degrees(lon2_rad)

    # Normalizar a longitude para ficar entre -180 e 180 graus
    lon2 = (lon2 + 540) % 360 - 180

    return lat2, lon2

if not drone.connected():
    exit()

drone.disable_pre_arm_checks()
drone.change_to_guided_mode()
drone.set_home(lat, lon)

drone.arm_drone()

drone.ascend(1.5)
# drone.move_forward(20)
# --home=33.2110977,-87.5367255,0,0

new_lat, new_lon = meters_to_geo(lat, lon, 3, 90)
drone.move_to_position(new_lat, new_lon)
measure_form(drone.current_altitude())
drone.return_to_home()

# drone.land()
# drone.disarm()


