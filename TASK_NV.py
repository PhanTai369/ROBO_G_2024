import cv2
import numpy as np
from simple_pid import PID

# Khởi tạo PID controller
pid = PID(1.0, 0.1, 0.05, setpoint=0)


def adjust_speed(current_speed, target_speed):
    pid.setpoint = target_speed
    speed_adjustment = pid(current_speed)
    return current_speed + speed_adjustment


def control_speed(current_speed, target_speed, max_speed):
    adjusted_speed = adjust_speed(current_speed, target_speed)
    if adjusted_speed > max_speed:
        adjusted_speed = max_speed
    return adjusted_speed


def process_frame(frame):
    # Chuyển đổi sang ảnh xám
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Làm mờ ảnh
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Phát hiện cạnh
    edges = cv2.Canny(blurred, 50, 150)

    # Tìm kiếm các đường viền
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Chọn đường viền lớn nhất (giả định là đường line)
        c = max(contours, key=cv2.contourArea)
        # Tính toán tâm của đường line
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # Vẽ tâm của đường line lên khung hình
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            # Hiển thị tọa độ
            cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return frame, (cx, cy)
    return frame, None


def main():
    cap = cv2.VideoCapture(0)  # Mở camera
    target_speed = 10  # Tốc độ mục tiêu khi robot lên dốc
    max_speed = 20  # Tốc độ tối đa để tránh bị tuột dốc
    current_speed = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Xử lý khung hình để xác định đường line và tọa độ
        frame, coordinates = process_frame(frame)

        # Hiển thị khung hình đã xử lý
        cv2.imshow('Frame', frame)

        if coordinates:
            print(f"Coordinates: {coordinates}")

        # Điều chỉnh tốc độ dựa trên tọa độ đường line
        if coordinates:
            # Điều chỉnh tốc độ và hướng đi tùy thuộc vào tọa độ đường line
            # current_speed = read_speed_from_sensor()  # Đọc tốc độ hiện tại từ cảm biến
            current_speed = control_speed(current_speed, target_speed, max_speed)
            # set_motor_speed(current_speed)  # Cập nhật tốc độ của động cơ

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
