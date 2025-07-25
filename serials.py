import serial
import time
import json

import json
import paho.mqtt.client as mqtt
# Set up serial connection
time.sleep(2)  # Wait for the connection to initialize
# Always close the serial connection when done



class myMqtt:
    def __init__(self) -> None:
        self.ser = serial.Serial('COM5', 9600)  # Change 'COM3' to your ESP32's port
        self.mqttc = mqtt.Client()
        self.mqttc.on_message = self.on_message
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_subscribe = self.on_subscribe# Use current time for initialization


    def on_connect(self, mqttc, obj, reason_code, properties):
        mqttc.subscribe("data/addAngle", 0)
        print("Connected to %s:%s" % (mqttc._host, mqttc._port))

    def on_message(self, mqttc, obj, msg):
        data = json.loads(msg.payload.decode())
        print(data)

        servo = data["addAngle"]
        data = json.dumps({"servo" : servo})
        self.ser.write(data.encode('utf-8'))
        # try:
        #     while True: 
                
        #         time.sleep(1)  # Wait for the connection to initialize
        # if self.ser.in_waiting > 0:  # Check if there is data waiting to be read
        #     line = self.ser.readline().decode('utf-8').rstrip()
        #     print(f"Received: {line}")
        # except KeyboardInterrupt:
        #     print("Stopped.")
        # except Exception :
        #     print("Error")
        # finally:
        #     ser.close()  
            

    def on_subscribe(self, mqttc, obj, mid, reason_code_list):
        print("Subscribed: " + str(mid) + " " + str(reason_code_list))
       

    def on_log(self, mqttc, obj, level, string):
        print(string)
        
    def setForm(self, form):
        self.form = form
ser = serial.Serial('COM5', 9600)
while True:
    print("Sending")
    data = json.dumps({"servo" : 90})
    ser.write(data.encode('utf-8'))
    time.sleep(1)
mymqtt = myMqtt()

mqttc = mymqtt.mqttc
mqttc.connect_async("192.168.1.105", 1883)
mqttc.subscribe("sensor/data", 0)
mqttc.loop_forever()




