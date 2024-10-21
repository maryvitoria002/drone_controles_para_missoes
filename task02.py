from app.drone.Drone import Drone
# import floripa.Camera as measure
from app.drone.tools.GPS import GPS

drone = Drone()
gps = GPS()

if not drone.connected():
    exit()

def run():
    try:
        lat, lon, _ = drone.get_gps_position()
        drone.change_to_guided_mode()
        drone.set_home(lat, lon)

        drone.arm_drone()

        drone.ascend(2)

        new_lat, new_lon = gps.meters_to_geo(lat, lon, 2, 90)
        drone.move_to_position(-14.3033883, -42.6944204)
        # measure.wait_to_continue(drone.current_altitude())
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


