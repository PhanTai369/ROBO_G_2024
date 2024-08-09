import cv2
import numpy as np
from ugot import ugot
import time
from simple_pid import PID

# Khởi tạo robot
got = ugot.UGOT()
got.initialize('10.220.5.228')
got.open_camera()

# Cài đặt thông số PID
pid = PID(1, 0.1, 0.05, setpoint=0)
pid.output_limits = (-100, 100)


# Hàm điều khiển servo
def control_servo(id, angle, duration):
    got.turn_servo_angle(id, angle, duration)
    time.sleep(duration)


# Hàm nhận diện đường line
def detect_line(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    return edges


# Hàm xử lý QR
def process_qr(frame):
    detector = cv2.QRCodeDetector()
    data, bbox, _ = detector.detectAndDecode(frame)
    if bbox is not None:
        nparr = np.array(bbox, dtype=np.int32)
        cv2.polylines(frame, [nparr], True, (0, 255, 0), 2)
        if data:
            print(f"QR Code detected: {data}")
            return data
    return None


# Hàm nhận diện màu
def detect_color(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    return mask


# Hàm di chuyển robot dựa trên nhận diện line và PID
def move_robot_based_on_line():
    frame = got.read_camera_data()
    if frame is not None:
        nparr = np.frombuffer(frame, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        edges = detect_line(img)
        midpoint = img.shape[1] // 2

        left_weight = np.sum(edges[:, :midpoint])
        right_weight = np.sum(edges[:, midpoint:])
        error = left_weight - right_weight
        control = pid(error)

        left_speed = 100 - control
        right_speed = 100 + control
        got.move(left_speed, right_speed)


# Hàm di chuyển qua đường hẹp góc vuông hoặc đường cong hình chữ S
def move_through_narrow_or_curved_path():
    while True:
        frame = got.read_camera_data()
        if frame is not None:
            nparr = np.frombuffer(frame, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Sử dụng hàm nhận diện line để điều khiển robot
            edges = detect_line(img)
            midpoint = img.shape[1] // 2

            left_weight = np.sum(edges[:, :midpoint])
            right_weight = np.sum(edges[:, midpoint:])
            error = left_weight - right_weight
            control = pid(error)

            # Điều chỉnh tốc độ và hướng dựa trên điều kiện góc vuông hoặc đường cong
            if abs(error) > 50:  # Nếu sai số lớn, tức là có thể đang ở đoạn đường cong hoặc góc vuông
                left_speed = 80 - control
                right_speed = 80 + control
            else:
                left_speed = 100 - control
                right_speed = 100 + control

            got.move(left_speed, right_speed)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Hàm di chuyển qua cầu với kiểm soát tốc độ
def move_over_bridge():
    while True:
        frame = got.read_camera_data()
        if frame is not None:
            nparr = np.frombuffer(frame, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Giả sử có thể sử dụng hàm detect_line để phát hiện độ dốc
            edges = detect_line(img)
            slope = np.mean(np.gradient(edges))

            if slope > 0.5:  # Nếu độ dốc lớn, điều chỉnh tốc độ
                got.move(120, 120)
            else:
                got.move(100, 100)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


# Hàm xử lý khi gặp đường cụt
def handle_dead_end():
    while True:
        frame = got.read_camera_data()
        if frame is not None:
            nparr = np.frombuffer(frame, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            edges = detect_line(img)
            # Giả sử phát hiện đường cụt dựa trên sự biến mất của line
            if np.sum(edges) < 1000:  # Nếu số lượng pixel của line quá ít, có thể là đường cụt
                print("Dead end detected, stopping and reversing...")
                got.stop()
                time.sleep(1)
                got.move(-100, -100)  # Đi lùi
                time.sleep(2)
                got.stop()
                break


# Hàm nhận diện đích đến
def detect_destination():
    while True:
        frame = got.read_camera_data()
        if frame is not None:
            nparr = np.frombuffer(frame, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Giả sử đích đến được nhận diện bằng một màu hoặc hình dạng đặc trưng
            mask = detect_color(img)
            if np.sum(mask) > 5000:  # Nếu nhận diện được một vùng lớn màu đỏ chẳng hạn
                print("Destination detected, stopping...")
                got.stop()
                break


# Hàm chính điều khiển robot thực hiện các nhiệm vụ
def main():
    try:
        # Nhiệm vụ 1: Di chuyển qua đường hẹp hoặc đường cong
        move_through_narrow_or_curved_path()

        # Nhiệm vụ 2: Di chuyển qua cầu
        move_over_bridge()

        # Nhiệm vụ 3: Xử lý khi gặp đường cụt
        handle_dead_end()

        # Nhiệm vụ 4: Nhận diện đích đến
        detect_destination()

        # Các nhiệm vụ khác...
        while True:
            frame = got.read_camera_data()
            if frame is not None:
                nparr = np.frombuffer(frame, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                # Nhận diện mã QR
                qr_data = process_qr(img)
                if qr_data:
                    print(f"Đã nhận diện được mã QR: {qr_data}")

                # Nhận diện màu
                mask = detect_color(img)
                cv2.imshow("Color Detection", mask)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        # Đảm bảo dừng robot khi kết thúc chương trình
        got.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
