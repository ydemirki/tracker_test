from pymavlink import mavutil

pixhawk = mavutil.mavlink_connection('/dev/serial0', baud=57600)
pixhawk.wait_heartbeat()
print("Bağlantı sağlandı!")

def force_arm():
    print("Zorla ARM ediliyor...")
    pixhawk.mav.command_long_send(
        pixhawk.target_system,
        pixhawk.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        1, 1, 21196, 0, 0, 0, 0, 0
    )
    print("Force ARM komutu gönderildi.")

def set_servo(channel, pwm_value):
    pixhawk.mav.command_long_send(
        pixhawk.target_system,
        pixhawk.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, channel, pwm_value, 0, 0, 0, 0, 0
    )
    print(f"Servo {channel} kanalına {pwm_value} PWM sinyali gönderildi.")

force_arm()
set_servo(9, 1500)
