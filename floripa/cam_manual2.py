import cv2 as cv
import numpy as np
import math
from math import pi
import threading
import os
from app.camera.Camera import Camera
from app.drone.Drone import Drone

# Configurar opções para o FFMPEG
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

# Constantes Globais
REFERENCE_PIXELS = 118
REFERENCE_SIZE_CM = 10
REFERENCE_DISTANCE_CM = 63.5

class VideoStreamThread:
    def __init__(self, src):
        self.cap = cv.VideoCapture(src, cv.CAP_FFMPEG)
        if not self.cap.isOpened():
            print("Não foi possível abrir o fluxo de vídeo.")
            self.stopped = True
            return

        self.ret = False
        self.frame = None
        self.stopped = False
        threading.Thread(target=self.update, args=()).start()

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                print("Falha ao ler o frame da câmera.")
                self.stopped = True

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()

class ShapeDetector:
    def __init__(self):
        pass

    @staticmethod
    def detect_shape(contour):
        epsilon = 0.04 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        num_vertices = len(approx)

        if num_vertices == 3:
            return "Triangle"
        elif num_vertices == 4:
            (_, _, w, h) = cv.boundingRect(approx)
            aspect_ratio = w / float(h)
            if 0.95 <= aspect_ratio <= 1.05:
                return "Square"
            else:
                return "Rectangle"
        elif num_vertices == 5:
            return "Pentagon"
        elif num_vertices == 6:
            return "Hexagon"
        else:
            area = cv.contourArea(contour)
            perimeter = cv.arcLength(contour, True)
            if perimeter == 0:
                return "Unknown"
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity > 0.8:
                return "Circle"
            else:
                return "Polygon"

class Measurement:
    def __init__(self, reference_pixels, reference_size_cm, reference_distance_cm):
        self.reference_pixels = reference_pixels
        self.reference_size_cm = reference_size_cm
        self.reference_distance_cm = reference_distance_cm
        self.cm_per_pixel_ref = self.reference_size_cm / self.reference_pixels

    def calculate_cm_per_pixel_current(self, current_distance_cm):
        return self.cm_per_pixel_ref * (current_distance_cm / self.reference_distance_cm)

    def convert_area_px2_to_cm2(self, area_pixels, cm_per_pixel_current):
        return area_pixels * (cm_per_pixel_current ** 2)

    def convert_perimeter_px_to_cm(self, perimeter_pixels, cm_per_pixel_current):
        return perimeter_pixels * cm_per_pixel_current

class ImageProcessor:
    def __init__(self):
        pass

    @staticmethod
    def preprocess_image(image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # blurred = cv.GaussianBlur(gray, (4, 4), 0)
        equalized = cv.equalizeHist(gray)
        alpha = 1.5  # Contraste
        beta = 50    # Brilho
        adjusted = cv.convertScaleAbs(equalized, alpha=alpha, beta=beta)
        edges = cv.Canny(adjusted, 50, 170)
        kernel = np.ones((5, 5), np.uint8)
        edges = cv.dilate(edges, kernel, iterations=1)
        # edges = cv.erode(edges, None, iterations=1)
        return edges

def main():
    # Inicialização
    drone = Drone()
    # URL RTSP da câmera
    rtsp_url = "rtsp://admin:admin@192.168.0.113:554/11"
    video_stream = VideoStreamThread(rtsp_url)
    image_processor = ImageProcessor()
    shape_detector = ShapeDetector()
    measurement = Measurement(REFERENCE_PIXELS, REFERENCE_SIZE_CM, REFERENCE_DISTANCE_CM)

    if not drone.connected():
        print("Drone não conectado")
        return

    if video_stream.stopped:
        print("Não foi possível iniciar o fluxo de vídeo.")
        return

    count = 0
    while not video_stream.stopped:
        count += 1
        # Atualiza a distância atual em cada iteração
        current_distance_m = drone.get_rangefinder_distance()
        if current_distance_m is None or current_distance_m <= 0:
            print("Distância inválida obtida do rangefinder.")
            continue
        current_distance_cm = current_distance_m * 100  # Converte para centímetros

        # Calcula cm por pixel atual
        cm_per_pixel_current = measurement.calculate_cm_per_pixel_current(current_distance_cm)

        # Leitura do frame da câmera
        ret, frame = video_stream.read()
        if not ret or frame is None:
            print("Falha ao ler o frame da câmera.")
            continue

        # Pré-processamento da imagem
        edges = image_processor.preprocess_image(frame)

        # Detecção de contornos
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area_pixels = cv.contourArea(contour)
            if area_pixels < 50:
                continue  # Ignora pequenos ruídos

            # Cálculo da área em cm²
            area_cm2 = measurement.convert_area_px2_to_cm2(area_pixels, cm_per_pixel_current)

            # Detecção da forma
            shape = shape_detector.detect_shape(contour)
            if shape:
                x, y, w, h = cv.boundingRect(contour)
                cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv.putText(frame, shape, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # Cálculo do perímetro em cm
                perimeter_pixels = cv.arcLength(contour, True)
                perimeter_cm = measurement.convert_perimeter_px_to_cm(perimeter_pixels, cm_per_pixel_current)

                # Exibição dos resultados
                size_text = f"Area: {area_cm2:.2f} cm², Perimeter: {perimeter_cm:.2f} cm"
                cv.putText(frame, size_text, (x, y + h + 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Exibição das imagens
        cv.imshow("Shape Detection with Size Estimation", frame)
        # cv.imshow("Edges", edges)
 
        # Verificação de saída
        if cv.waitKey(1) & 0xFF == ord('q'):
            cv.imwrite(r"C:\Users\bruno\educa_drone\floripa-tasks\floripa\file\print.png".format(count), frame)
            break
        
        if cv.waitKey(1) & 0xFF == ord('p'):
            cv.imwrite(r"C:\Users\bruno\educa_drone\floripa-tasks\floripa\file\print{}.png".format(count), frame)

    # Liberação de recursos
    video_stream.stop()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
