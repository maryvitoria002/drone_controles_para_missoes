import cv2 as cv
import numpy as np
from math import pi
from app.camera.Camera import Camera
from app.drone.Drone import Drone
import math

drone = Drone()
camera = Camera()
camera_type = "rtsp"
camera.initialize_video_capture(camera_type)

if not drone.connected():
    print("Drone not connected")
    exit()

def detect_shape(contour):
    epsilon = 0.04 * cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)
    
    if len(approx) == 3:
        return "Triangle"
    elif len(approx) == 4:
        (_, _, w, h) = cv.boundingRect(approx)
        aspect_ratio = w / float(h)
        if 0.95 <= aspect_ratio <= 1.05:
            return "Square"
        else:
            return "Rectangle"
    elif len(approx) == 5:
        return "Pentagon"
    elif len(approx) == 6:
        return "Hexagon"
    elif len(approx) > 6:
        area = cv.contourArea(contour)
        perimeter = cv.arcLength(contour, True)
        circularity = (4 * np.pi * area) / (perimeter * perimeter)
        if circularity > 0.8:
            return "Circle"
        return "Polygon"

reference_pixels = 300  
reference_size_cm = 46   
reference_distance = 63.5 # cm  
current_distance_cm = 30

def calculate_real_size(pixels_detected, current_distance_cm):
    real_size =  (reference_size_cm / reference_pixels) * pixels_detected * (current_distance_cm / reference_distance)
    return real_size

def convert_area_px2_to_cm2(area_pixels, reference_pixels, reference_size_cm, reference_distance_cm, current_distance_cm):
    if reference_pixels <= 0 or reference_distance_cm <= 0 or current_distance_cm <= 0:
        print("Erro: Valores de referência inválidos para o cálculo da área.")
        return None

    cm_per_pixel_ref = reference_size_cm / reference_pixels
    cm_per_pixel_current = cm_per_pixel_ref * (current_distance_cm / reference_distance_cm)

    # print(f"cm_per_pixel_ref: {cm_per_pixel_ref}")
    # print(f"cm_per_pixel_current: {cm_per_pixel_current}")
    # print(f"area_pixels: {area_pixels}")

    if cm_per_pixel_current <= 0:
        print("Erro: cm_per_pixel_current é menor ou igual a zero.")
        return None

    try:
        area_cm2 = area_pixels * (cm_per_pixel_current ** 2)
    except OverflowError as e:
        print(f"Erro de Overflow ao calcular area_cm2: {e}")
        return None

    return area_cm2

def convert_perimeter_px_to_cm(perimeter_pixels, reference_pixels, reference_size_cm, reference_distance_cm, current_distance_cm):
    cm_per_pixel_ref = reference_size_cm / reference_pixels
    cm_per_pixel_current = cm_per_pixel_ref * (current_distance_cm / reference_distance_cm)
    perimeter_cm = perimeter_pixels * cm_per_pixel_current
    return perimeter_cm

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

def get_area(shape, real_size_cm_x, real_size_cm_y):
    match shape:
        case "Square":
            area = real_size_cm_x * real_size_cm_y
        case "Rectangle":
            area = real_size_cm_x * real_size_cm_y
        case "Circle":
            area = pi * ((real_size_cm_x / 2) ** 2)
        case "Triangle":
            area = (real_size_cm_x * real_size_cm_y) / 2
        case "Pentagon":
            area = (real_size_cm_x * real_size_cm_y) * 0.4301
        case "Hexagon":
            area = (real_size_cm_x * real_size_cm_y) * 0.6495
        case "Polygon":
            area = (real_size_cm_x * real_size_cm_y) * 0.78 
        case _:
            area = 0            
    return area

def main():
    count = 0

    while camera.cap.isOpened():
        current_distance_m = drone.get_rangefinder_distance()
        if current_distance_m is None:
            continue
        else:
            current_distance_cm = current_distance_m * 100  # Converte metros para centímetros

        # Verifica se a distância atual é válida
        if current_distance_cm <= 0:
            print("Distância inválida obtida do rangefinder.")
            continue

        print(f"current_distance_cm: {current_distance_cm}")

        count += 1
        ret, frame = camera.read_capture()
        frame = cv.flip(frame, 1)
        if not ret:
            continue

        edges = preprocess_image(frame)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
        for contour in contours:

            area = cv.contourArea(contour)
            if area < 50:
                continue

            area_cm2 = convert_area_px2_to_cm2(area, reference_pixels, reference_size_cm, reference_distance, current_distance_cm)
            
            if area_cm2 is None:
                continue

            shape = detect_shape(contour)
            if shape:
                x, y, w, h = cv.boundingRect(contour)
                cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv.putText(frame, shape, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            
                perimeter_cm = convert_perimeter_px_to_cm(cv.arcLength(contour, True), reference_pixels, reference_size_cm, reference_distance, current_distance_cm)
        
                size_text = f"Area: {area_cm2:.2f}, Perimeter: {perimeter_cm:.2f}"
                cv.putText(frame, size_text, (x, y + h + 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        cv.imshow("Shape Detection with Size Estimation", frame)
        cv.imshow("Edges", edges)
    
        if count % 20 == 0:
            cv.imwrite(r"C:\Users\bruno\educa_drone\floripa-tasks\floripa\file\print.png", frame)
    
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    camera.cap.release()
    cv.destroyAllWindows()



