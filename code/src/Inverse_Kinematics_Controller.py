import serial
import time
import numpy as np
from ikpy.chain import Chain

Robot_Chain = Chain.from_urdf_file("URDF_FIVE_DOF.urdf",active_links_mask=[False, True, True, True, True, True, False])

ser = serial.Serial('COM6', 115200) 
time.sleep(2)
def wait_for_arduino():
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Received: {response}")
            if response == "Awaiting Orders":
                break

def send_command(angle1, angle2, angle3, angle4, angle5, gripper):
    command = f"{angle1} {angle2} {angle3} {angle4} {angle5} {gripper}\n"
    print(f"Sent: {command.strip()}")
    ser.write(command.encode())
    time.sleep(0.1)
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Received: {response}")
            if response == "Done":
                break

def get_user_input():
    while True:
        print("=======================================================================================")
        user_input = input("Enter target position (x y z Gripper_state) separated by spaces (q\Q to Quit): ")
        try:
            if user_input.strip().lower() == 'q':
                print("Exiting program.")
                if ser.is_open:
                    send_command(0, 90, 90, 90, 90, 1)  # Parks the Robot
                    print("Robot parked. Closing serial connection.")
                    ser.close()
                exit(0)
            parts = user_input.strip().split()
            x, y, z, gripper_state = map(float, parts)
            if not gripper_state.is_integer() or gripper_state not in [0, 1]:
                raise ValueError("Gripper state must be 0 (Open) or 1 (Close).")
            if not (-200 <= x <= 200 and -200 <= y <= 200 and 0 <= z <= 300):
                raise ValueError("Coordinates must be within realistic operational limits: x and y between -200 and 200, z between 0 and 300.")
            print(f"Target Position: x={x}, y={y}, z={z}, Gripper State: {'Open' if gripper_state == 0 else 'Closed'}")
            print("=======================================================================================")
            return [x, y, z, int(gripper_state)]
        except ValueError as ve:
            print(f"Error : {ve}")

if __name__ == "__main__":
    wait_for_arduino()

    while True:
        # Get target position from user input
        target_position = get_user_input()       # Contains [x, y, z, gripper_state]
        gripper_state = int(target_position[3])  # Gripper state (0 for closed, 1 for open)
        Target_Orientation = [0, 0, -1]          # Orientation (adjust as needed)

        # Compute inverse kinematics
        ik_results = Robot_Chain.inverse_kinematics(target_position[:3], Target_Orientation, orientation_mode="Y")
        joint_angles_deg = np.rad2deg(ik_results)

        # Map joint angles to motor angles (adjust based on your robot's configuration)
        Motor_Angle_Degrees = [0, 0, 0, 0, 0]
        Motor_Angle_Degrees[0] = joint_angles_deg[1]
        Motor_Angle_Degrees[1] = joint_angles_deg[2] + 90
        Motor_Angle_Degrees[2] = joint_angles_deg[3] + 90
        Motor_Angle_Degrees[3] = joint_angles_deg[4] + 90
        Motor_Angle_Degrees[4] = joint_angles_deg[5] + 90
        Motor_Angle_Degrees = np.round(Motor_Angle_Degrees, 1)

        # Display computed angles
        print("Computed Motor Angles (Degrees):")
        print(f"Motor 1 = {Motor_Angle_Degrees[0]} | Motor 2 = {Motor_Angle_Degrees[1]} | Motor 3 = {Motor_Angle_Degrees[2]} | Motor 4 = {Motor_Angle_Degrees[3]} | Motor 5 = {Motor_Angle_Degrees[4]};")
        print(f"Gripper State: {'Open' if gripper_state == 0 else 'Closed'}")
        print("=======================================================================================")

        # Wait for user confirmation to execute
        execute = input("Press 'e' to execute or any other key to recompute angles: ").strip().lower()
        if execute == 'e':
            send_command(Motor_Angle_Degrees[0], Motor_Angle_Degrees[1], Motor_Angle_Degrees[2], Motor_Angle_Degrees[3], Motor_Angle_Degrees[4], gripper_state)
            print("Command executed!")
        else:
            print("Angles recomputed. Enter a new target position.")