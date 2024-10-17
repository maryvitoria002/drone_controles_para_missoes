from app.drone.Drone import Drone
from app.drone.tools.GPS import GPS

drone = Drone()
gps = GPS()

lat, lon, alt = drone.get_gps_position()

if not drone.connected():
    exit()

def run():
    try:
        drone.change_to_guided_mode()
        drone.set_home(lat, lon)

        drone.arm_drone()

        drone.ascend(1.5)

        new_lat, new_lon = gps.meters_to_geo(lat, lon, 2, 90)
        drone.move_to_position(new_lat, new_lon)
        drone.return_to_home()

    except KeyboardInterrupt:
        drone.land()
        drone.disarm()
        exit()

    except Exception as e:
        print(e)
        drone.land()
        drone.disarm()

while True:
    msg = drone.conn.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=True)
    if msg.chan6_raw:
        if msg.chan6_raw >= 2014:
            print("Canal 6 detectado.")
            run()
            break


