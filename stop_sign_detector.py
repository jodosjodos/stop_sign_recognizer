import cv2
import numpy as np
import time
import threading
from datetime import datetime


class StopSignDetector:
    def __init__(self, serial_port=None, baud_rate=9600):
        """
        Initialize the stop sign detector without requiring Arduino

        Args:
            serial_port (str or None): Serial port for Arduino (use None to disable UART)
            baud_rate (int): Baud rate for serial communication
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.arduino = None
        self.stop_detected = False
        self.stop_start_time = None
        self.stop_duration = 3.0  # 3 seconds

        # UART is disabled if serial_port is None
        if serial_port:
            self.init_serial()
        else:
            print("UART communication disabled (no Arduino)")

        # Load stop sign classifier (or fallback)
        try:
            self.stop_cascade = cv2.CascadeClassifier("stop_sign_classifier.xml")
            if self.stop_cascade.empty():
                raise ValueError("Cascade load failed")
            print("✅ Custom stop sign classifier loaded")
        except:
            self.stop_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            )
            print("⚠️ Fallback: Using face detection as dummy classifier")

        # Red color detection in HSV
        self.lower_red1 = np.array([0, 50, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 50, 50])
        self.upper_red2 = np.array([180, 255, 255])

        # Start UART thread
        self.uart_thread = threading.Thread(
            target=self.uart_communication_loop, daemon=True
        )
        self.uart_thread.start()

    def init_serial(self):
        """Try initializing serial port (if Arduino connected)"""
        try:
            import serial

            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)  # Give time to connect
            print(f"✅ Serial connected to {self.serial_port}")
        except Exception as e:
            print(f"⚠️ Serial error: {e}")
            print("🔌 UART will be simulated with console logs")
            self.arduino = None

    def detect_stop_sign_color(self, frame):
        """Detect red shapes (approx octagons)"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if 6 <= len(approx) <= 10:
                    x, y, w, h = cv2.boundingRect(contour)
                    aspect_ratio = w / float(h)
                    if 0.7 <= aspect_ratio <= 1.3:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(
                            frame,
                            "STOP SIGN (RED)",
                            (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )
                        return True
        return False

    def detect_stop_sign_cascade(self, frame):
        """Detect stop sign using Haar cascade (fallback uses faces)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        signs = self.stop_cascade.detectMultiScale(gray, 1.1, 5, minSize=(30, 30))
        for x, y, w, h in signs:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(
                frame,
                "STOP SIGN (Cascade)",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2,
            )
        return len(signs) > 0

    def uart_communication_loop(self):
        """Sends UART signal or prints log every 100ms"""
        while True:
            now = time.time()
            if self.stop_detected and self.stop_start_time:
                if now - self.stop_start_time < self.stop_duration:
                    self.send_uart_signal(1)
                else:
                    self.stop_detected = False
                    self.stop_start_time = None
                    self.send_uart_signal(0)
            else:
                self.send_uart_signal(0)
            time.sleep(0.1)

    def send_uart_signal(self, signal):
        """Send UART signal if connected, or simulate with logs"""
        if self.arduino:
            try:
                self.arduino.write(str(signal).encode())
                print(f"[UART] Sent: {signal}")
            except Exception as e:
                print(f"[UART ERROR] {e}")
        else:
            log_time = datetime.now().strftime("%H:%M:%S")
            if signal == 1:
                print(f"[{log_time}] Simulated UART: 🚦 STOP")
            elif signal == 0:
                print(f"[{log_time}] Simulated UART: ✅ MOVE")

    def process_frame(self, frame):
        """Process frame to detect stop sign, update status text"""
        cascade = self.detect_stop_sign_cascade(frame)
        color = self.detect_stop_sign_color(frame)
        detected = cascade or color

        if detected and not self.stop_detected:
            self.stop_detected = True
            self.stop_start_time = time.time()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] 🚨 STOP SIGN DETECTED!")

        status = "STOPPING" if self.stop_detected else "MOVING"
        status_color = (0, 0, 255) if self.stop_detected else (0, 255, 0)
        cv2.putText(
            frame,
            f"Status: {status}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            status_color,
            2,
        )

        if self.stop_detected:
            time_left = max(
                0, self.stop_duration - (time.time() - self.stop_start_time)
            )
            cv2.putText(
                frame,
                f"Remaining: {time_left:.1f}s",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )

        return frame, detected

    def run(self):
        """Main loop"""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Could not open webcam")
            return

        print("🎥 Stop Sign Detection Started")
        print("Press 's' to simulate detection")
        print("Press 'q' to quit")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame, _ = self.process_frame(frame)
                cv2.imshow("Stop Sign Detection", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    # Simulate detection
                    self.stop_detected = True
                    self.stop_start_time = time.time()
                    print("[Simulated] STOP triggered manually")

        finally:
            cap.release()
            cv2.destroyAllWindows()
            print("🔚 Program exited.")


def main():
    """
    Main entry to test system without Arduino.
    Set serial_port=None to disable UART hardware.
    """
    detector = StopSignDetector(serial_port=None)
    detector.run()


if __name__ == "__main__":
    main()
