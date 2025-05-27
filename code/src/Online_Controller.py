from flask import Flask, render_template, request, jsonify
import serial
import time
import sys

# Initialize Flask app
app = Flask(__name__)

# Initialize serial communication
try:
    ser = serial.Serial('COM6', 115200)
    time.sleep(2)
except serial.SerialException as e:
    print(f"Error: Could not open serial port: {e}")
    sys.exit(1)

def wait_for_arduino():
    try:
        while True:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"Received: {response}")
                if response == "Awaiting Orders":
                    break
    except Exception as e:
        print(f"Error while waiting for Arduino: {e}")

def send_command(angle1, angle2, angle3, angle4, angle5, gripper):
    try:
        command = f"{angle1} {angle2} {angle3} {angle4} {angle5} {gripper}\n"
        print(f"Sent: {command.strip()}")
        ser.write(command.encode())
        
        # Wait for a response from the Arduino
        while True:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                print(f"Received: {response}")
                if response == "Done":
                    break
    except Exception as e:
        print(f"Error while sending command: {e}")

# Route for the main control page
@app.route('/')
def index():
    return render_template('index.html')

# Route to handle slider and button data
@app.route('/send_command', methods=['POST'])
def handle_command():
    try:
        data = request.json
        angle1 = data['angle1']
        angle2 = data['angle2']
        angle3 = data['angle3']
        angle4 = data['angle4']
        angle5 = data['angle5']
        gripper = data['gripper']
        
        # Send the command to the robot
        send_command(angle1, angle2, angle3, angle4, angle5, gripper)
        return jsonify({"status": "Command sent successfully!"})
    except Exception as e:
        return jsonify({"status": f"Error: {e}"}), 500

if __name__ == '__main__':
    try:
        wait_for_arduino()
        # Disable Flask's auto-reload by setting use_reloader=False
        app.run(host='0.0.0.0', port=5000,debug=True, use_reloader=False)
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")