# -*- coding: utf-8 -*-
import cv2
import numpy as np
import serial
import time

# PID 控制参数（可根据实际场景调整）
KP = 0.1    # 比例系数
KI = 0    # 积分系数
KD = 0    # 微分系数
pid_integral = 0  # 积分项
last_error = 0    # 上一次偏差，用于计算微分项
TARGET_X = 320
ser = serial.Serial(
    port='/dev/ttyAMA0',  
    baudrate=115200,
    timeout=0.1
)


def order_points(pts):
    
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]  
    rect[2] = pts[np.argmax(s)]  
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  
    rect[3] = pts[np.argmax(diff)]  
    return rect


def four_point_transform(image, pts):
    rect = order_points(pts)
    (tl, tr, br, bl) = rect
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
    return warped, M, maxWidth, maxHeight


def send_motor_command(mode, direction, subdivision, data, speed):
    
    command = bytearray([0x7B, 0x01])

   
    command.append(mode)

    
    command.append(direction)

    
    subdivision_byte = 0
    if subdivision == 2:
        subdivision_byte = 0x02
    elif subdivision == 4:
        subdivision_byte = 0x04
    elif subdivision == 8:
        subdivision_byte = 0x08
    elif subdivision == 16:
        subdivision_byte = 0x10
    elif subdivision == 32:
        subdivision_byte = 0x20
    command.append(subdivision_byte)

    
    command.append((data >> 8) & 0xFF)  
    command.append(data & 0xFF)  

    
    command.append((speed >> 8) & 0xFF)  
    command.append(speed & 0xFF)  

   
    bcc = 0
    for byte in command:
        bcc ^= byte
    command.append(bcc)

    
    command.append(0x7D)

    
    try:
        if ser.is_open:
            ser.write(command)
            time.sleep(0.01)  
            return True
        else:
            print("not open")
            return False
    except Exception as e:
        print(f"uart send error: {e}")
        return False


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('cam not open')
        exit()

 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, -4)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 100])
    kernel = np.ones((5, 5), np.uint8)

    try:
       
        send_motor_command(0x01, 1, 32, 0, 12)

        while True:
            ret, frame = cap.read()
            if not ret:
                print('no frame')
                continue

            orig_frame = frame.copy()
            #orig_frame = orig_frame[80,80,540,380]
            hsv = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_black, upper_black)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

            valid_contours = []
            for c in contours:
                area = cv2.contourArea(c)
                if area > 3000:
                    peri = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                    if len(approx) == 4:
                    
                        rect_pts = order_points(approx.reshape(4, 2))
                        (tl, tr, br, bl) = rect_pts

                    
                        width_top = np.linalg.norm(tr - tl)
                        width_bottom = np.linalg.norm(br - bl)
                        height_left = np.linalg.norm(bl - tl)
                        height_right = np.linalg.norm(br - tr)

                        width = (width_top + width_bottom) / 2
                        height = (height_left + height_right) / 2

                        if min(width, height) < 1e-5:
                            continue

                        aspect_ratio = width / height
                        if 1.2 < aspect_ratio < 1.6:
                            valid_contours.append(approx)

            for c in valid_contours:
            
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)

           
                try:
                  
                    pts = c.reshape(4, 2).astype(np.float32)
                    warped, M_perspective, width_w, height_w = four_point_transform(orig_frame, pts)

          
                    center_warped = np.array([[[width_w / 2, height_w / 2]]], dtype=np.float32)

        
                    M_inv = np.linalg.inv(M_perspective)
                    center_orig = cv2.perspectiveTransform(center_warped, M_inv)[0][0]

          
                    center_orig = (int(center_orig[0]), int(center_orig[1]))
                    cv2.circle(frame, center_orig, 2, (255, 0, 0), -1) 

                    coord_text = f"({center_orig[0]}, {center_orig[1]})"
                    cv2.putText(frame, coord_text, (center_orig[0] + 20, center_orig[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)  

                   
                    #if center_orig[0] > 340:
                        
                      #  send_motor_command(0x01, 0, 32, 0, 10)
                    #elif center_orig[0] < 300:
                        
                     #   send_motor_command(0x01, 1, 32, 0, 10)
                    #else:
                        
                    #    send_motor_command(0x01, 1, 32, 0, 0)
                    
                    # ---------------- PID 控制逻辑 ----------------
                    # 1. 计算偏差（当前 x 与目标 x 的差）
                    error = center_orig[0] - TARGET_X  
                    # 2. 计算积分项
                    pid_integral += error  
                    # 3. 计算微分项
                    pid_derivative = error - last_error  
                    # 4. 计算 PID 输出
                    pid_output = KP * error + KI * pid_integral + KD * pid_derivative  

                    # 5. 限制输出范围（根据步进电机实际可接受的速度范围调整）
                    pid_output = np.clip(pid_output, -12, 12)  

                    # 6. 输出给步进电机（这里简单映射为速度，方向根据正负判断）
                    if pid_output > 0:
                        direction = 0  # 假设 1 是正方向
                    else:
                        direction = 1  # 假设 0 是反方向
                    speed = int(abs(pid_output))
                    send_motor_command(0x01, direction, 32, 0, speed)

                    # 7. 更新上一次偏差
                    last_error = error  
                

                    print(f"PID Output: {pid_output}, Speed: {speed}, Direction: {direction}")
                except Exception as e:
                    print(f"Processing error: {e}")
              

           
            cv2.imshow('frame',frame)
            cv2.imshow('mask',mask)

            
            key = cv2.waitKey(1)
            if key == 27:
                break

    finally:
       
        send_motor_command(0x01, 1, 32, 0, 0)

        cap.release()
        cv2.destroyAllWindows()
        ser.close()  
        print("0")
