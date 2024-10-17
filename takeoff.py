from app.drone.Drone import Drone

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
        drone.ascend(1)
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


while True:
    msg = drone.conn.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=True)
    if msg.chan6_raw:
        if msg.chan6_raw >= 2014:
            print("Canal 6 detectado.")
            run()
            break

