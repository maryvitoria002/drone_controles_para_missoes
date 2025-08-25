from app.drone.Drone import Drone
import time
from app.drone.tools.GPS import GPS
gps = GPS()



#Instanciando a classe

drone = Drone()

if not drone.connected():
    print("O drone não conectado veyr")
    exit()

#Definindo uma função

def run():
    try:
        lat, lon, _ = drone.get_gps_position()
        drone.set_home(lat, lon)
        drone.change_to_guided_mode()
        drone.arm_drone()
        drone.takeoff(5)
        time.sleep(5)
        new_lat, new_lon = gps.meters_to_geo(lat, lon, 4, 180)
        drone.move_to_position(new_lat, new_lon)
        drone.descend(3)
        time.sleep(5)
        drone.land()
        drone.disarm()

     #Se a gente parar com algum comando no teclado tipo ctrl + c aí o drone pousa

    except KeyboardInterrupt:
        drone.land()
        drone.disarm()
        exit()

    #Se der algum erro ou interrupção além da ultima ai mostra o erro e pousa

    except Exception as e:
        print(e)
        drone.land()
        drone.disarm()
        exit()

#Rodando as funções

run()

