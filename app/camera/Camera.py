import time
import cv2
from app.drone.Drone import Drone
from app.files.FileManager import FileManager
from app.aruco.ArucoDetector import ArucoDetector
from app.files.Reports import DetectionsByAltReport
# from app.Ssh import SSHConnection


class CameraConnection:
    source = None
    
    def connection(self):
        print("Inicializando captura de vídeo...")
        return cv2.VideoCapture(int(self.source) if self.source.isdigit() else self.source)
        
class CameraRTSP(CameraConnection):
    source = 'rtsp://admin:admin@192.168.0.113:554/11'

class CameraComputer(CameraConnection):
    source = '0'

class CameraIMX519(CameraConnection):
    source = 'tcp://192.168.0.101:8554'
    
    # def connection(self):
        # host = '192.168.0.101'  # Substitua pelo endereço do seu servidor
        # port = 22  # Porta SSH
        # username = 'drone'  # Substitua pelo seu nome de usuário
        # password = 'Dron3s'  # Substitua pela sua senha
        # command = 'libcamera-vid -t 0 --inline --listen -o tcp://0.0.0.0:8554 --nopreview'

        # sshConn = SSHConnection(host, port, username, password)
        # sshConn.ssh_command(command)
        # return CameraConnection.connection(self)

class CameraESP32CAM(CameraConnection):
    source = 'http://192.168.0.115:81/stream'

class CameraAnalog(CameraConnection):
    source = '/dev/video2'
    
class CameraDroidCAM(CameraConnection):
    source = 'http://10.70.196.179:4747/video'
class CameraConnectionFactory():
    
    @staticmethod
    def create(type) -> CameraConnection: 
        cameras = {
            'rtsp' : CameraRTSP,
            'computer': CameraComputer,
            'imx': CameraIMX519,
            'analog': CameraAnalog,
            'esp32': CameraESP32CAM,
            'droid': CameraDroidCAM
        }
           
        return cameras[type]()

class Camera:
    def __init__(self) -> None:
      
        self.cap = None
        self.ret = None
        self.frame = None

    def initialize_video_capture(self, type):
        camera = CameraConnectionFactory.create(type)
        
        self.cap = camera.connection()
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError("Falha ao abrir a captura de vídeo. Verifique a conexão com a câmera.")
        else:
            print("Captura de vídeo inicializada com sucesso.")
        
    def read_capture(self):
        self.ret = None
        self.frame = None
        self.ret, self.frame = self.cap.read()
        return self.ret, self.frame

       
    def capture_images_and_metadata(self, drone: Drone,  fileManager: FileManager, aruco_detector: ArucoDetector, alt, type, num_images=10, pause_time=0.5):
       
        for i in range(num_images):
            ret, frame = self.cap.read()
            processed_image, ids, __ = aruco_detector.detect_arucos(frame)
            lat, long, alt_ = drone.get_gps_position()
            
            if ret:
                fileManager.create_image(frame, alt, i)  
                fileManager.create_meta_data(lat, long, alt, drone.current_altitude(), i)
        
        self.clean_buffer()

        self.release_video_capture() 
                    
    def clean_buffer(self):
        for _ in range(60):  
            self.cap.grab()

    def save_image(self, path):
        cv2.imwrite(path, self.frame)
    
    def release_video_capture(self): # Libera a câmera
        if self.cap:
            self.cap.release()
            self.cap = None
            print("Captura de vídeo liberada.")

    def display_video(self):
        # Verifica se a captura de vídeo foi inicializada corretamente
        if self.cap is None or not self.cap.isOpened():
            print("Erro: Captura de vídeo não inicializada.")
            return
        
        # Loop para exibir o vídeo capturado
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Falha ao capturar o frame. Verifique a câmera.")
                break
            
            # Exibe o frame capturado
            if self.frame is not None and self.frame.size > 0:
                print("Iniciando exibição de video da câmera")
                #cv2.imshow("Camera Video", self.frame)
            else:
                print("Falha ao capturar o frame. Verifique a câmera.")
            
            # Destroi todas as janelas quando finalizar
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
