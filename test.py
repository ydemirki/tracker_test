from pymavlink import mavutil

class PyMavlinkHelper:
    def __init__(self, connection_string: str):
        self.vehicle = mavutil.mavlink_connection(connection_string,baud=57600)
        self.vehicle.wait_heartbeat()

    def force_arm(self) -> None:
        """
        Force arm the drone. This will send the ARMED command without checking if the drone is armable.
        """
        try:
            # Send the arm command directly to force the drone to arm
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,  # ARM the vehicle
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            print("Force arming the drone.")
            
            # Wait for acknowledgment
            ack_msg = try_recv_match(self.vehicle, message_name="COMMAND_ACK")
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Drone force armed.")
            else:
                print("Failed to force arm the drone.")
        except Exception as e:
            print(f"Error during force arm: {e}")

    def disarm(self) -> None:
        """
        Disarm the drone.
        """
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                0,  # DISARM the vehicle
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            print("Drone disarmed.")
            
            # Wait for acknowledgment
            ack_msg = try_recv_match(self.vehicle, message_name="COMMAND_ACK")
            if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Drone disarmed successfully.")
            else:
                print("Failed to disarm the drone.")
        except Exception as e:
            print(f"Error during disarm: {e}")

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
            custom_mode = 0  # Default value for custom_mode, this can be adjusted based on specific needs.
            
            # Send mode change command
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mode_id,
                custom_mode  # Include the custom_mode argument
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
helper = PyMavlinkHelper('/dev/serial0')  # Replace with correct connection string

# Force arm the drone
helper.force_arm()

# Set mode to GUIDED
helper.set_mode('STABILIZE')

# Set servo 1 to PWM value 1500 (neutral position for most servos)
helper.set_servo(6, 1500)

# Disarm the drone
helper.disarm()
