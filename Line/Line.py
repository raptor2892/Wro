import cv2
import numpy as np
import serial
import time

from line_constants import (
    ALIGNMENT_THRESHOLD_PX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    DARK_LINE_HSV_MAX,
    DARK_LINE_HSV_MIN,
    DEFAULT_BAUD_RATE,
    DEFAULT_CAMERA_INDEX,
    DEFAULT_SERIAL_PORT,
    EXIT_KEY,
    LEFT_TURN_ERROR_THRESHOLD_PX,
    LINE_DETECTION_RATIO,
    MASK_KERNEL_SIZE,
    MASK_POINTS,
    MAX_FRAMES_WITHOUT_LINE,
    RIGHT_TURN_ERROR_THRESHOLD_PX,
    SIGNAL_ROW_FRACTION,
)

class LineFollower:
    def __init__(self, camera_index=DEFAULT_CAMERA_INDEX, serial_port=DEFAULT_SERIAL_PORT, baud_rate=DEFAULT_BAUD_RATE):
        self.camera = cv2.VideoCapture(camera_index)
        self.serial_conectado = False
        self.serial_port = serial_port
        self.baud_rate = baud_rate

        self.skip_mode = False
        self.skip_frames = 0
        self.last_movement = "A"
        self.frames_without_line = 0
        self.max_frames_without_line = MAX_FRAMES_WITHOUT_LINE
        self.search_mode = False
        
        # Alignment uses point 3 (center)
        self.alignment_threshold = ALIGNMENT_THRESHOLD_PX

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        # HSV range for dark line detection
        self.lower_hsv = np.array(DARK_LINE_HSV_MIN)
        self.upper_hsv = np.array(DARK_LINE_HSV_MAX)

        self.setup_serial_connection()

    def setup_serial_connection(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.serial_conectado = True
            print("Arduino connected successfully")
        except Exception as e:
            self.serial_conectado = False
            print(f"Arduino not detected: {e}")
            print("Simulation mode...")

    def send_command(self, command):
        """Send a command to Arduino."""
        if self.serial_conectado:
            try:
                self.arduino.write(command.encode())
                print(f"Command sent: {command}")
            except Exception as e:
                print(f"Error sending command: {e}")
        else:
            print(f"Simulated command: {command}")

    def process_frame(self, frame):
        """Convert the frame to HSV and build a mask for the line."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Remove noise
        kernel = np.ones(MASK_KERNEL_SIZE, np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def get_line_centers(self, mask, points=MASK_POINTS):
        """Get the line center across several horizontal rows."""
        height, width = mask.shape
        positions = []
        step = height // (points + 1)

        for index in range(1, points + 1):
            y = index * step
            row = mask[y, :]
            indices = np.where(row > 0)[0]
            if len(indices) > 0:
                center_x = int(np.mean(indices))
                positions.append((center_x, y))
            else:
                positions.append(None)
        return positions

    def calculate_error(self, positions, frame_width):
        """Calculate the error using detected points."""
        center_x = frame_width // 2
        valid_points = [point[0] for point in positions if point is not None]

        if len(valid_points) == 0:
            return None

        average_x = int(np.mean(valid_points))
        error = average_x - center_x
        normalized_error = (error / center_x) * 100
        return normalized_error

    def check_point3_alignment(self, positions, frame_width):
        """
        Check whether point 3 (center) is aligned with the frame center.
        Returns: (aligned: bool, pixel_error: int)
        """
        center_x = frame_width // 2
        
        # Point 3 is index 2 (0-indexed)
        point3 = positions[2] if len(positions) > 2 else None
        
        if point3 is None:
            return False, None
        
        pixel_error = point3[0] - center_x
        aligned = abs(pixel_error) <= self.alignment_threshold
        
        return aligned, pixel_error

    def decide_movement_with_alignment(self, positions, frame_width):
        """
        Decide movement by prioritizing point 3 alignment.
        If point 3 is aligned, move forward. Otherwise, correct.
        """
        aligned, pixel_error = self.check_point3_alignment(positions, frame_width)
        
        if pixel_error is None:
            return None, None, None  # No point 3 detected
        
        if aligned:
            # Point 3 aligned = move forward
            return "A", pixel_error, True
        else:
            # Correct based on point 3 error
            if pixel_error < LEFT_TURN_ERROR_THRESHOLD_PX:  # Line is to the left
                return "I", pixel_error, False
            elif pixel_error > RIGHT_TURN_ERROR_THRESHOLD_PX:  # Line is to the right
                return "D", pixel_error, False
            else:
                return "A", pixel_error, False  # Minimal error, move forward

    def detect_horizontal_line(self, mask):
        "Detect horizontal line (signal to move straight)."
        height, width = mask.shape
        y = int(height * SIGNAL_ROW_FRACTION)
        row = mask[y, :]
        white_pixels = np.sum(row > 0)

        return white_pixels > (LINE_DETECTION_RATIO * width)

    def run(self):
        """Main loop."""
        print("Starting line follower (point 3 alignment)...")
        print("Press 'q' to exit")
        print("\nArduino commands:")
        print("A = Forward | D = Right | I = Left | S = Stop | R = Reverse")
        print("\nMode: The robot aligns using the detected point 3 (center)")

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("Error capturing frame")
                break

            frame = cv2.flip(frame, 1)
            mask = self.process_frame(frame)
            positions = self.get_line_centers(mask, points=MASK_POINTS)

            # Detect horizontal line
            if self.detect_horizontal_line(mask):
                self.skip_mode = True
                self.skip_frames = 20
                cv2.putText(frame, "HORIZONTAL LINE - MOVING FORWARD", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # SKIP MODE: move forward until another line is found
            if self.skip_mode:
                command = "A"
                self.skip_frames -= 1
                if self.skip_frames <= 0:
                    self.skip_mode = False
                cv2.putText(frame, "MODE: Moving straight", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                alignment_state = "Skipping line"
            
            # NORMAL MODE: align with point 3
            else:
                command, pixel_error, aligned = self.decide_movement_with_alignment(positions, frame.shape[1])
                
                if command is None:
                    # No point 3 detected
                    self.frames_without_line += 1
                    
                    if self.frames_without_line > self.max_frames_without_line:
                        command = "R"  # Reverse
                        alignment_state = "No line - reversing"
                        cv2.putText(frame, "MODE: Searching for line", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    else:
                        command = self.last_movement
                        alignment_state = "Waiting for line..."
                        cv2.putText(frame, "Waiting for line...", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    self.frames_without_line = 0
                    
                    # Show alignment state
                    if aligned:
                        alignment_state = "ALIGNED"
                        text_color = (0, 255, 0)
                    else:
                        alignment_state = f"Correcting ({pixel_error:+.0f}px)"
                        text_color = (0, 165, 255)
                    
                    cv2.putText(frame, f"Point 3: {alignment_state}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
                    
                    # Show pixel error
                    cv2.putText(frame, f"Error: {pixel_error:+.0f} px", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Send command to Arduino
            if command:
                self.send_command(command)
                if command in ["A", "D", "I"]:
                    self.last_movement = command

            # Draw detected points
            center_x = frame.shape[1] // 2
            for index, point in enumerate(positions):
                if point is not None:
                    # Point 3 (index 2) is highlighted
                    if index == 2:
                        # Green if aligned, red if not
                        _, pixel_error = self.check_point3_alignment(positions, frame.shape[1])
                        if pixel_error is not None and abs(pixel_error) <= self.alignment_threshold:
                            color = (0, 255, 0)  # Green = aligned
                            thickness = 8
                        else:
                            color = (0, 0, 255)  # Red = misaligned
                            thickness = 8
                        
                        # Draw larger circle for point 3
                        cv2.circle(frame, point, thickness, color, -1)
                        cv2.putText(frame, "P3", (point[0] + 12, point[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    else:
                        # Other points in standard color
                        color = (0, 255, 255)
                        cv2.circle(frame, point, 5, color, -1)
                        cv2.putText(frame, str(index + 1), (point[0] + 10, point[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            # Center reference line
            cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)
            
            # Tolerance zone (alignment threshold)
            cv2.line(frame, (center_x - self.alignment_threshold, 0), 
                     (center_x - self.alignment_threshold, frame.shape[0]), (0, 255, 0), 1)
            cv2.line(frame, (center_x + self.alignment_threshold, 0), 
                     (center_x + self.alignment_threshold, frame.shape[0]), (0, 255, 0), 1)

            # Show current command
            cv2.putText(frame, f"Command: {command}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Line Follower - Point 3", frame)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF == EXIT_KEY:
                self.send_command("S")  # Stop robot
                break

        self.release_resources()

    def release_resources(self):
        """Release resources."""
        self.camera.release()
        cv2.destroyAllWindows()
        if self.serial_conectado:
            self.arduino.close()
            print("Arduino disconnected")

if __name__ == "__main__":
    robot = LineFollower(
        camera_index=DEFAULT_CAMERA_INDEX,
        serial_port=DEFAULT_SERIAL_PORT,
        baud_rate=DEFAULT_BAUD_RATE
    )
    robot.run()