import serial
import time


class ServoControl:
    def __init__(self, port, baudrate, timeout, angle=40, increment=10):
        """
        Constructor for the class.
        :param port: Serial port, e.g., '/dev/arduino'
        :param baudrate: Baud rate, e.g., 115200
        :param timeout: Timeout for serial communication
        :param angle: Initial angle of the servo (default is 40)
        :param increment: Angle increment for servo rotation (default is 10)
        """
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for the serial connection to establish
            print(f"Connected to {port} at {baudrate} baud.")
            self.angle = angle
            self.increment = increment
        except serial.SerialException as e:
            print(f"Failed to open port {port}: {e}")
            self.serial_port = None

    def set_angle(self):
        """
        Set angle of the servo and increment the angle for the next command.
        """
        if not self.serial_port or not self.serial_port.is_open:
            print("Serial port is not open!")
            return

        # Check if the angle is within the valid range (40 to 140)
        if 40 <= self.angle <= 140:
            command = f"<SERVO:{self.angle}>\n"  # Construct the servo command
            try:
                self.serial_port.write(command.encode())
                print(f"Command sent: {command.strip()}")

                # Adjust the increment to reverse direction if limits are reached
                if (
                    self.angle + self.increment > 140
                    or self.angle + self.increment < 40
                ):
                    self.increment = -self.increment

                self.angle += self.increment  # Update angle for next time
            except serial.SerialException as e:
                print(f"Failed to send command: {e}")
        else:
            print("Invalid angle. Please provide a value between 40 and 140.")

    def close(self):
        """
        Close the serial connection.
        """
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")


if __name__ == "__main__":
    # Create a servo control object
    servo = ServoControl("/dev/ttyUSB0", 115200, 1)

    try:
        # Continuously set the servo angle in a loop
        while True:
            servo.set_angle()
            time.sleep(2)  # Add a small delay to avoid flooding the serial port
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the serial connection when exiting
        servo.close()
        print("Exiting...")
