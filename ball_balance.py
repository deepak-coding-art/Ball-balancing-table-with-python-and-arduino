import cv2
import numpy as np
import serial
import time

# Initalize sirial communication
ser = serial.Serial('COM5', baudrate = 115200, timeout = 1)
time.sleep(3)


# Read Video.
frame_width = 640
frame_hight = 480
cap = cv2.VideoCapture(1) 
cap.set(3, frame_width)
cap.set(4, frame_hight)
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame', frame_width, frame_hight)

errer_Ki_x = 0
errer_Ki_y = 0
last_errer_x = 0
last_errer_y = 0

def send_data(value_1, value_2):
    value_1 = chr(value_1)
    value_2 = chr(value_2)
    while ser.inWaiting:
        ok = ser.read()
        if (ok == b'k'):
            break
    ser.write(str.encode(value_1))
    ser.write(str.encode(value_2))

def map_value(val, in_min, in_max, out_min, out_max):
    val = int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    if(out_min <= val <= out_max):
        return val
    elif(val < out_min):
        return out_min
    elif(val > out_max):
        return out_max

def Track_Ball(img):
    a = b = r = 0
    # Convert to grayscale. 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Blur using 3 * 3 kernel. 
    gray_blurred = cv2.blur(gray, (3, 3)) 
      
    # Apply Hough transform on the blurred image. 
    detected_circles = cv2.HoughCircles(gray_blurred,  
                       cv2.HOUGH_GRADIENT, 1, int(frame_hight), param1 = 50, 
                   param2 = 30, minRadius = int(frame_hight/10), maxRadius = int(frame_hight/3)) 
      
    # Draw circles that are detected. 
    if detected_circles is not None: 
      
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 
      
        for pt in detected_circles[0, :]: 
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle. 
            cv2.circle(img, (a, b), r, (0, 255, 0), 2) 
      
            # Draw a small circle (of radius 1) to show the center. 
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3)

    # Map values
    maped_x = map_value(a, 0, 640, 0, 127)
    maped_y = map_value(b, 0, 480, 0, 127)
    
    return maped_x, maped_y, r, img;       

def calculate_PID(current_x, current_y):
    global errer_Ki_x
    global errer_Ki_y
    global last_errer_x
    global last_errer_y
    # PID gain's
    Kp = 0.8
    Ki = 0.25
    Kd = 0.1
    
    
    # Desired position of ball
    set_point_x = 63
    set_point_y = 63
    Ki_minvalue = 15
    
    # Calculate error of ball's position
    current_errer_x = set_point_x - current_x
    current_errer_y = set_point_y - current_y
    
    # Limite the integral component
    if current_errer_x < Ki_minvalue & current_errer_x != 0:
        errer_Ki_x += current_errer_x
    else:
        errer_Ki_x = 0
    if current_errer_y < Ki_minvalue & current_errer_y != 0:
        errer_Ki_y += current_errer_y
    else:
        errer_Ki_y = 0
    if errer_Ki_x > 90:
        errer_Ki_x = 90
    if errer_Ki_y > 90:
        errer_Ki_y = 90
    if errer_Ki_x < -90:
        errer_Ki_x = -90
    if errer_Ki_y < -90:
        errer_Ki_y = -90
        
    # Limite th derivative component
    if current_errer_x == 0:
        daravitive_x = 0
    if current_errer_y == 0:
        daravitive_y = 0
        
    # Calculate proportional, integral, derivative components
    perpotnal_x = current_errer_x * Kp
    perpotnal_y = current_errer_y * Kp
    intigral_x = errer_Ki_x * Ki
    intigral_y = errer_Ki_y * Ki
    daravitive_x = (current_errer_x - last_errer_x) * Kd
    daravitive_y = (current_errer_y - last_errer_y) * Kd

    # Find last errer
    last_errer_x = current_errer_x
    last_errer_y = current_errer_y

    # Sum the PID components for output value
    output_x = perpotnal_x + intigral_x + daravitive_x
    output_y = perpotnal_y + intigral_y + daravitive_y

    return output_x, output_y;

while True:
    _, frame = cap.read()
    x_code, y_code, radius, ball_image = Track_Ball(frame)
    x_angle, y_angle = calculate_PID(x_code, y_code)
    x_angle = map_value(x_angle, -90, 90, 0, 127)
    y_angle = map_value(y_angle, -90, 90, 0, 127)
    send_data(x_angle, y_angle)
    cv2.imshow('frame', ball_image)
    print(x_angle, y_angle)
    if cv2.waitKey(1) & 0xFF == 27:
        break
cap.release()
cv2.destroyAllWindows()
