# Import socket module
import socket
import cv2
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import disp

global sendBack_angle, sendBack_Speed, current_speed, current_angle, MAX_SPEED, MIN_SPEED, MAX_ANGLE, MIN_ANGLE
sendBack_angle = 0
sendBack_Speed = 0
current_speed = 0
current_angle = 0
MAX_SPEED = 150
MIN_SPEED = -150
MAX_ANGLE = 25
MIN_ANGLE = -25


# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the port on which you want to connect
PORT = 54321
# connect to the server on local computer
s.connect(('host.docker.internal', PORT))


def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed

def canny(image):
    """
    Hàm tiền xử lý:
    Yêu cầu:
    +Chuyển ảnh RGB về Gray
    +Khử nhiễu
    +Sử dụng thuật toán Canny của openCV để nhận diện các cạnh
    """
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny_image = cv2.Canny(blur, 50, 150)
    #cv2.imshow("Canny", canny_image)
    return canny_image

def region_of_interest(image):
    """
    Hàm cắt ảnh
    Yêu cầu:
    +Tạo đa giác để cắt ảnh giữ lại các đường kẻ trên mặt đường
    """
    #print("Tao da giac chua phan anh can xu ly")
    height, width = image.shape
    polygons = np.array([[(0, 210), (0, height), (width, height), (width, 210), (320, 130)]])
    #print(polygons)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    #cv2.imshow("Mask", mask)
    return masked_image

def display_line(lines, image):
    """
    Hàm vẽ đoạn thẳng
    Yêu cầu:
    +Lấy tọa độ (x1, y1, x2, y2) từ một line trong mảng lines
    +Từ đó kẻ các đoạn thẳng có điểm đầu điểm cuối vào image 
    """
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            #print(line)
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(lines_image, (x1, y1), (x2, y2), (255,0,0), 2)
    #cv2.imshow("Lines Image", lines_image)
    return lines_image

def intercept_of_lines(line1, line2):
    """
    Hàm tìm giao điểm của hai đoạn thẳng cho trước
    Yêu cầu:
    +Nếu hai đoạn thẳng trùng nhau thì trả về trung điểm của đoạn thứ 2 (đoạn ta lấy tham chiếu)
    +Viết phương trình đoạn thẳng y = ax + b cho mỗi đoạn
    +Giao điểm tìm được phải nằm trong hai đoạn thẳng nói trên nếu không sẽ trả về None
    """
    x1, y1, x2, y2 = line1
    X1, Y1, X2, Y2 = line2
    if (x1 == x2) or (X1 == X2):
        return None
    #Tìm hệ số của phương trình đường thẳng
    a1 = (y2 - y1) / (x2 - x1)
    b1 = y1 - a1*x1
    a2 = (Y2 - Y1) / (X2 - X1)
    b2 = Y1 - a2*X1
    x, y = 0, 0
    if (a1 == a2):
        x, y = ((line2[0]+line2[1])/2, (line2[2]+line2[3])/2) 
    else:
        x = (b2-b1)/(a1-a2)
        y = a2*x + b2;
    if (x <= max(X1, X2) and x >= min(X1, X2)) and (y <= max(y1, y2) and y >= min(y1, y2)):
        return (x, y)
    return None

def display_intercepts(left_line, right_line, lines, image):
    combo_image = image
    left_points = []
    right_points = []
    if lines is not None:
        for line in lines:
            line_ = line.reshape(4)
            left_point = intercept_of_lines(line_, left_line)
            right_point = intercept_of_lines(line_, right_line)
            if left_point != None:
                left_points.append(left_point)
            if right_point != None:
                right_points.append(right_point)

    
    left_points = np.array(left_points, dtype=np.int32)
    #print(left_points)
    right_points = np.array(right_points, dtype = np.int32)

    #print(right_points)
    if left_points.size != 0:
        averaged_left_point = np.mean(left_points, axis = 0, dtype = np.int32)
        cv2.circle(combo_image, tuple(averaged_left_point), 1, (0, 0, 255), 2)
    else:
        averaged_left_point = None

    
    if right_points.size != 0:

        averaged_right_point = np.mean(right_points, axis = 0, dtype = np.int32)
        cv2.circle(combo_image, tuple(averaged_right_point), 1, (0, 0, 255), 2)

    else:
        averaged_right_point = None
    
    
    # print(combo_image, averaged_left_point, averaged_right_point)


    #cv2.imshow("Points Image", combo_image)
    return combo_image, averaged_left_point, averaged_right_point

if __name__ == "__main__":
    try:
        error_left = 0
        error_right = 0
        alpha = 0
        while True:

            """
            - Chương trình đưa cho bạn 1 giá trị đầu vào:
                * image: hình ảnh trả về từ xe
                * current_speed: vận tốc hiện tại của xe
                * current_angle: góc bẻ lái hiện tại của xe
            - Bạn phải dựa vào giá trị đầu vào này để tính toán và
            gán lại góc lái và tốc độ xe vào 2 biến:
                * Biến điều khiển: sendBack_angle, sendBack_Speed
                Trong đó:
                    + sendBack_angle (góc điều khiển): [-25, 25]
                        NOTE: ( âm là góc trái, dương là góc phải)
                    + sendBack_Speed (tốc độ điều khiển): [-150, 150]
                        NOTE: (âm là lùi, dương là tiến)
            """

            message_getState = bytes("0", "utf-8")
            s.sendall(message_getState)
            state_date = s.recv(100)

            try:
                current_speed, current_angle = state_date.decode(
                    "utf-8"
                    ).split(' ')
            except Exception as er:
                print(er)
                pass

            message = bytes(f"1 {sendBack_angle} {sendBack_Speed}", "utf-8")
            s.sendall(message)
            data = s.recv(100000)

            try:
                image = cv2.imdecode(
                    np.frombuffer(
                        data,
                        np.uint8
                        ), -1
                    )


                print(current_speed, current_angle)
                print(image.shape)
                

                
                # your process here
                #masked_image = region_of_interest(image)
                width = image.shape[1]
                height = image.shape[0]
                #Kiểm tra kích thước của ảnh
                #print(image.shape)
                lane_image = np.copy(image)
                #Hai thanh điều hướng
                left_line = (100, 200, 300, 200)                      #Thanh trái
                right_line = (340, 200, 540, 200)                     #Thanh phải                                      


                canny_image = canny(lane_image)
                
                cropped_image = region_of_interest(canny_image)
                lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength = 40, maxLineGap = 5)
                lines_image = display_line(lines, lane_image)

                #cv2.imshow("Line Image", lines_image)
                combo_image = cv2.addWeighted(lane_image, 0.8, lines_image, 1, 1)
                
                cv2.line(combo_image, (100, 200), (300, 200), (23, 238, 253), 1)
                cv2.line(combo_image, (540, 200), (340, 200), (23, 238, 253), 1)

                combo_image, left_intercept, right_intercept = display_intercepts(left_line, right_line, lines, combo_image)

                
                #Bên trái
                a_left = MAX_ANGLE / (left_line[0] - left_line[2])
                b_left = MAX_ANGLE - a_left * (320 - left_line[2])
                #Bên phải
                a_right = MIN_ANGLE / (right_line[0] - right_line[2])
                b_right = MIN_ANGLE - a_right * (right_line[0] - 320)
                
                last_error_left = error_left
                last_error_right = error_right

                if left_intercept is not None:
                    error_left = left_intercept[0] - 320
                    alpha_left = a_left * (-error_left) + b_left
                else:
                    error_left = left_line[0] - 320
                    alpha_left = 0
                if right_intercept is not None:
                    error_right = (right_intercept[0] - 320)
                    alpha_right = a_right * (error_right) + b_right
                else:
                    error_right = right_line[2] - 320
                    alpha_right = 0
                
                alpha_proportional = alpha_left + alpha_right
                alpha = alpha_proportional - alpha * 0.1
                print(f'left_error = {error_left}, right_error = {error_right}\nalpha_proportional = {alpha_proportional}')
                if abs(alpha) <= 1.5:
                    sendBack_Speed = 40
                    sendBack_Angle = 0
                elif MAX_ANGLE - abs(alpha) < 2:
                    sendBack_Speed = -40
                    sendBach_Angle = alpha
                else:
                    sendBack_Speed = 30
                    sendBack_Angle = alpha
                cv2.waitKey(1)
                # Control(angle, speed)
                Control(sendBack_Angle, sendBack_Speed)

            except Exception as er:
                print(er)
                pass

    finally:
        print('closing socket')
        s.close()