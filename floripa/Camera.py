import cv2 as cv
import numpy as np
from math import pi
from app.camera.Camera import Camera


camera = Camera()
camera_type = "droid"
camera.initialize_video_capture(camera_type)

def detect_shape(contour):
    epsilon = 0.04 * cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)
    
    if len(approx) == 3:
        return "Triangle"
    elif len(approx) == 4:
        (x, y, w, h) = cv.boundingRect(approx)
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
        if circularity > 0.75:
            return "Circle"
        return "Polygon"

reference_pixels = 300  
reference_size_cm = 46   
reference_distance = 106
current_distance_cm = 30

def calculate_real_size(pixels_detected, current_distance_cm):
    real_size =  (reference_size_cm / reference_pixels) * pixels_detected * (current_distance_cm / reference_distance)
    return real_size

def preprocess_image(image):
    # gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # blurred = cv.GaussianBlur(gray, (4, 4), 0)
    edges = cv.Canny(image, 50, 170)
    edges = cv.dilate(edges, None, iterations=2)
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
    
    return area

def main(distance = 0):
    current_distance_cm = distance * 100 if distance else 30
    count = 0
    
    while camera.cap.isOpened() and count < 100:
        count += 1
        ret, frame = camera.read_capture()
        frame = cv.flip(frame, 1)
        if not ret:
            continue

        edges = preprocess_image(frame)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv.contourArea(contour)
            if area < 500 or area > 5000:
                continue
            
            shape = detect_shape(contour)
            if shape:
                x, y, w, h = cv.boundingRect(contour)
                cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
                cv.putText(frame, shape, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                
                real_size_cm_x = calculate_real_size(w, current_distance_cm)
                real_size_cm_y = calculate_real_size(h, current_distance_cm)
                
                area = get_area(shape, real_size_cm_x, real_size_cm_y)
            
                size_text = f"({real_size_cm_x:.2f}, {real_size_cm_y:.2f}) {area:.2f}"
                cv.putText(frame, size_text, (x, y + h + 20), cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
        
        cv.imshow("Shape Detection with Size Estimation", frame)
        
        if count % 10 == 0:
            cv.imwrite(r"C:\Users\bruno\educa_drone\imav-tasks\floripa\file\print.png", frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    camera.cap.release()
    cv.destroyAllWindows()
    
