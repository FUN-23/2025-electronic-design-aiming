# -*- coding: utf-8 -*-

import cv2
import numpy as np
import serial
import time
import RPi.GPIO as GPIO  # 导入GPIO库

# PID 控制参数
KP = 0.20
KP_y = 0.14
KI = 0
KD = 0
KD_y = 0
pid_integral = 0
last_error = 0
TARGET_X = 330
TARGET_Y = 237

# 按键配置（BCM编号17，对应物理引脚11）
BUTTON_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 上拉，低电平触发

# 初始化串口（X轴和Y轴）
ser_x = serial.Serial(
    port='/dev/ttyAMA3',
    baudrate=115200,
    timeout=0.1
)

ser_y = serial.Serial(
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


def send_motor_command(serial_port, mode, direction, subdivision, data, speed):
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
        if serial_port.is_open:
            serial_port.write(command)
            time.sleep(0.01)
            return True
        else:
            print(f"Serial port {serial_port.port} not open")
            return False
    except Exception as e:
        print(f"UART send error on {serial_port.port}: {e}")
        return False


def find_rectangle(frame, lower_black, upper_black, kernel):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_black, upper_black)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area = cv2.contourArea(c)
        if area > 2000:
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
                if 1.0 < aspect_ratio < 2.0:
                    return True, c, mask
    return False, None, mask


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('Camera not open')
        GPIO.cleanup()  # 释放GPIO资源
        exit()

    # 设置图像参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, -4)

    # 颜色检测参数
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 120])
    kernel = np.ones((5, 5), np.uint8)

    try:
        # 初始化电机
        send_motor_command(ser_x, 0x01, 1, 32, 0, 0)
        send_motor_command(ser_y, 0x01, 1, 32, 0, 0)
        time.sleep(1)

        # ================ 新增：等待按键按下 ================
        print("等待按键按下以启动矩形搜索...（按Ctrl+C退出）")
        while True:
            # 检测到按键按下（低电平）
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                print("按键已按下，开始搜索矩形...")
                time.sleep(0.2)  # 消抖
                break
            time.sleep(0.05)  # 降低CPU占用
        # ==================================================

        # 按键按下后，执行矩形搜索
        print("Searching for rectangle...")
        found = False
        rotation_count = 0
        max_rotations = 12
        while not found and rotation_count < max_rotations:
            # 读取图像并翻转
            ret, frame = cap.read()
            if not ret:
                print('No frame')
                continue
            frame = cv2.flip(frame, -1)  # 图像翻转

            # 尝试找到矩形
            found, contour, mask = find_rectangle(frame, lower_black, upper_black, kernel)

            if found:
                print("Rectangle found!")
                break
            else:
                rotation_count += 1
                print(f"Rectangle not found, rotating 90 degrees (attempt {rotation_count}/{max_rotations})")
                send_motor_command(ser_x, 0x02, 1, 32, 300, 100)
                time.sleep(0.3)

        if not found:
            print("Max rotation attempts reached, could not find rectangle")

        # 找到矩形后，进入跟踪模式
        while found:
            ret, frame = cap.read()
            if not ret:
                print('No frame')
                continue

            orig_frame = frame.copy()
            hsv = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, lower_black, upper_black)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            valid_contours = []
            succeed_flag = 0
            for c in contours:
                area = cv2.contourArea(c)
                if area > 2000:
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
                        if 1.1 < aspect_ratio < 1.9:
                            succeed_flag = 1
                            valid_contours.append(approx)

            # 若丢失矩形，重新搜索（保持原逻辑）
            if 0:
                print("Rectangle lost, starting search again...")
                found = False
                rotation_count = 0
                while not found and rotation_count < max_rotations:
                    ret, frame = cap.read()
                    if not ret:
                        print('No frame')
                        continue

                    found, contour, mask = find_rectangle(frame, lower_black, upper_black, kernel)

                    if found:
                        print("Rectangle found again!")
                        break
                    else:
                        rotation_count += 1
                        print(f"Rectangle not found, rotating 90 degrees (attempt {rotation_count}/{max_rotations})")
                        send_motor_command(ser_x, 0x02, 1, 32, 90, 200)
                        time.sleep(2)

                if not found:
                    print("Could not find rectangle again, exiting tracking")
                    break
                else:
                    continue

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
                    print({center_orig[0]}, {center_orig[1]})
                    cv2.putText(frame, coord_text, (center_orig[0] + 20, center_orig[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                    # X轴PID控制
                    error_x = center_orig[0] - TARGET_X
                    pid_integral_x = pid_integral + error_x
                    pid_derivative_x = error_x - last_error
                    pid_output_x = KP * error_x + KI * pid_integral_x + KD * pid_derivative_x
                    pid_output_x = np.clip(pid_output_x, -1000, 1000)

                    # Y轴PID控制
                    error_y = center_orig[1] - TARGET_Y
                    pid_integral_y = pid_integral + error_y
                    pid_derivative_y = error_y - last_error
                    pid_output_y = KP_y * error_y + KI * pid_integral_y + KD_y * pid_derivative_y

                    # 输出限幅
                    if error_y > 40 or error_y < -40:
                        pid_output_y = np.clip(pid_output_y * 0.6, -20, 20)
                    else:
                        pid_output_y = np.clip(pid_output_y, -6, 6)

                    if error_x > 40 or error_x < -40:
                        pid_output_x = np.clip(pid_output_x * 0.4, -50, 50)
                    else:
                        pid_output_x = np.clip(pid_output_x, -4, 4)

                    # 控制X轴电机
                    dir_x = 0 if pid_output_x > 0 else 1
                    speed_x = int(abs(pid_output_x))
                    send_motor_command(ser_x, 0x02, dir_x, 32, speed_x, 10)

                    # 控制Y轴电机
                    dir_y = 0 if pid_output_y > 0 else 1
                    speed_y = int(abs(pid_output_y))
                    send_motor_command(ser_y, 0x02, dir_y, 32, speed_y, 10)

                    # 更新误差
                    last_error = error_x

                    print(f"X: PID={pid_output_x}, Speed={speed_x}, Dir={dir_x}")
                    print(f"Y: PID={pid_output_y}, Speed={speed_y}, Dir={dir_y}")

                except Exception as e:
                    print(f"Processing error: {e}")

            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)

            key = cv2.waitKey(1)
            if key == 27:
                break

    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        # 停止电机并释放资源
        send_motor_command(ser_x, 0x01, 1, 32, 0, 0)
        send_motor_command(ser_y, 0x01, 1, 32, 0, 0)
        cap.release()
        cv2.destroyAllWindows()
        ser_x.close()
        ser_y.close()
        GPIO.cleanup()  # 清理GPIO资源
        print("System stopped")
        