from pymavlink import mavutil

class PyMavlinkHelper:
    def __init__(self, connection_string: str):
        self.vehicle = mavutil.mavlink_connection(connection_string)
        self.vehicle.wait_heartbeat()

    def arm(self) -> None:
        """
        Arm the drone. This will send the ARMED command.
        """
        if self.vehicle.is_armable:
            self.vehicle.arducopter_arm()  # This will arm the drone
            print("Drone armed.")
        else:
            print("Drone not armable.")

    def disarm(self) -> None:
        """
        Disarm the drone.
        """
        self.vehicle.arducopter_disarm()  # This will disarm the drone
        print("Drone disarmed.")
        
    def set_mode(self, mode: str) -> None:
        """
        Set the drone's mode.

        Args:
            mode (str): The mode to set for the drone (e.g., 'GUIDED', 'STABILIZE', 'LOITER', etc.).
        """
        try:
            mode_mapping = {
                "STABILIZE": 0,
                "ACRO": 1,
                "ALT_HOLD": 2,
                "AUTO": 3,
                "GUIDED": 4,
                "LOITER": 5,
                "RTL": 6,
                "CIRCLE": 7,
                "POSITION": 8,
                "LAND": 9,
                "OF_LOITER": 10,
                "TOY": 11,
                "FOLLOW": 12,
            }

            if mode not in mode_mapping:
                print(f"Invalid mode: {mode}")
                return

            mode_id = mode_mapping[mode]
            
            # Send mode change command
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mode_id
            )
            
            print(f"Setting drone mode to {mode}")
            
            # Wait for acknowledgment
            ack_msg = try_recv_match(self.vehicle, message_name="COMMAND_ACK")
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Drone mode set to {mode}")
            else:
                print(f"Failed to set drone mode to {mode}")
        except Exception as e:
            print(f"Error setting mode: {e}")
            
    def set_servo(self, servo_id: int, pwm_value: int) -> None:
        """
        Control a servo by setting the PWM value.

        Args:
            servo_id (int): The servo ID (e.g., 1 for the first servo).
            pwm_value (int): The PWM value to set (between 1000 and 2000).
        """
        if pwm_value < 1000 or pwm_value > 2000:
            print("PWM value must be between 1000 and 2000.")
            return
        
        # Send the servo control command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # Confirmation
            servo_id,  # Servo number (e.g., 1, 2, 3, etc.)
            pwm_value,  # PWM value to set
            0, 0, 0, 0, 0  # Unused parameters
        )
        
        print(f"Setting servo {servo_id} to PWM {pwm_value}")

# Helper function to handle receiving MAVLink messages
def try_recv_match(vehicle, message_name: str):
    try:
        msg = vehicle.recv_match(type=message_name, blocking=True)
        return msg
    except Exception as e:
        print(f"Error receiving message: {e}")
        return None


# Example usage:
helper = PyMavlinkHelper('udp:127.0.0.1:14551')

# Arm the drone
helper.arm()

# Set mode to GUIDED
helper.set_mode('GUIDED')

# Set servo 1 to PWM value 1500 (neutral position for most servos)
helper.set_servo(1, 1500)

# Disarm the drone
helper.disarm()
