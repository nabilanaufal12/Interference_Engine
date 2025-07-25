import socketio
import serial
import json
import time

ser = serial.Serial('COM7', 115200, timeout=0.1)
sio = socketio.Client()
    
@sio.event
def connect():
    print('Connection established to server')
    sio.emit("join", "Hallo")

@sio.event
def message(data):
    print('Message received from server:', data)


@sio.event
def disconnect():
    print('Disconnected from server')

def read_from_usb():
    try:
        if ser.in_waiting > 0:  # Mengecek apakah ada data masuk
            data = ser.readline().decode('utf-8').strip()
            return json.loads(data)
    except Exception as e:
        print(f"Error reading from USB: {e}")
    return None

def send_to_esp32(data):
    try:
        json_data = json.dumps(data)
        ser.write((json_data + '\n').encode())  
        print(f"Sent to ESP32: {json_data}")
    except Exception as e:
        print(f"Error sending to ESP32: {e}")
        
@sio.event
def addAngle(data):
    print('ANGLE DIUBAH MASBRO')
    send_to_esp32(data)
    
@sio.event
def setLats(data):
    print('DAPAT LATS')
    send_to_esp32(data)
    
@sio.event
def setLongs(data):
    print('DAPAT LONG')
    send_to_esp32(data)
    
@sio.event
def conf(data):
    send_to_esp32(data)
    
sio.connect('http://localhost:5000')

try:
    while True:
        esp_data = read_from_usb()
        if esp_data:
            sio.emit("data",esp_data)
            print(f"Sent data to server: {esp_data}")
            
        
except KeyboardInterrupt:
    print("Program stopped.")
finally:
    ser.close()  
    sio.disconnect()  
