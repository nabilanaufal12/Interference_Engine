import tkinter as tk
from tkinter import  ttk
from PIL import ImageTk, Image
from tkinter import StringVar
import time
from threading import Thread,Timer
import json

import cv2
class VideoMonitorApp(tk.Tk):
    def __init__(self, mqtt):
        super().__init__()
        self.title("GOERINDAM CYBER SEA - AVS MONITORING PANEL")
        self.geometry("1270x680")
        self.active = True
        
        
        # Bagian baris pertama untuk video monitoring
        self.video_frame_1 = tk.Label(self, text="Video 1", bg="gray", width=55, height=20)
        self.video_frame_2 = tk.Label(self, text="Video 2", bg="gray", width=55, height=20)
        self.video_frame_3 = tk.Label(self, text="Video 3", bg="gray", width=55, height=20)
        
        self.video_frame_1.grid(row=0, column=0, padx=10, pady=10)
        self.video_frame_2.grid(row=0, column=1, padx=10, pady=10)
        self.video_frame_3.grid(row=0, column=2, padx=10, pady=10)
        
        # Baris kedua, kolom pertama untuk detail koordinat dan informasi
        self.detail_frame = tk.Frame(self, bg="white", width=10, height=10)
        self.detail_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")
        
        # Data 1
        self.coordinate_label = tk.Label(self.detail_frame, text="Azimuth:")
        self.coordinate_label.grid(row=3, column=0,padx=5, pady=5, sticky="w")
        self.coordinate_label.config(background="white", font="Arial 10 bold")
        
        self.coordinate_value = StringVar()
        self.coordinate_value.set("(X: 0, Y: 0)")
        self.coordinate_value_label = tk.Label(self.detail_frame, textvariable=self.coordinate_value)
        self.coordinate_value_label.grid(row=3, column=1, columnspan=2,padx=5, pady=5, sticky="w")
        self.coordinate_value_label.config(width=40)
        
        #Data 2
        self.counter_label = tk.Label(self.detail_frame, text="Counter:")
        self.counter_label.grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.counter_label.config(background="white", font="Arial 10 bold")
        
        self.counter_value = StringVar()
        self.counter_value.set("0")
        self.counter_value_label = tk.Label(self.detail_frame, textvariable=self.counter_value)
        self.counter_value_label.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        self.counter_value_label.config(width=40)
        
        #Data 3
        self.lat = tk.Label(self.detail_frame, text="Lat:")
        self.lat.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.lat.config(background="white", font="Arial 10 bold")
        
        self.lat_value = StringVar()
        self.lat_value.set("0")
        self.LatValue = tk.Label(self.detail_frame, textvariable=self.lat_value)
        self.LatValue.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.LatValue.config(width=40)
        
        #Data 4
        self.long = tk.Label(self.detail_frame, text="Long:")
        self.long.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.long.config(background="white", font="Arial 10 bold")
        
        self.long_value = StringVar()
        self.long_value.set("0")
        self.LongValue = tk.Label(self.detail_frame, textvariable=self.long_value)
        self.LongValue.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.LongValue.config(width=40)
        
        #Data 5
        self.latdir = tk.Label(self.detail_frame, text="Lat Direction:")
        self.latdir.grid(row=4, column=0, padx=5, pady=5, sticky="w")
        self.latdir.config(background="white", font="Arial 10 bold")
        
        self.ld_value = StringVar()
        self.ld_value.set("0")
        self.latDirection = tk.Label(self.detail_frame, textvariable=self.ld_value)
        self.latDirection.grid(row=4, column=1, padx=5, pady=5, sticky="w")
        self.latDirection.config(width=40)
        
        #Data 6
        self.londir = tk.Label(self.detail_frame, text="Long Direction:")
        self.londir.grid(row=5, column=0, padx=5, pady=5, sticky="w")
        self.londir.config(background="white", font="Arial 10 bold")
        
        self.lgd_value = StringVar()
        self.lgd_value.set("0")
        self.longDirection = tk.Label(self.detail_frame, textvariable=self.lgd_value)
        self.longDirection.grid(row=5, column=1, padx=5, pady=5, sticky="w")
        self.longDirection.config(width=40)

        #Data 1.1
        self.lat2_label = tk.Label(self.detail_frame, text="Lat 2:")
        self.lat2_label.grid(row=0, column=3, padx=5, pady=5, sticky="w")
        self.lat2_label.config(background="white", font="Arial 10 bold")
        
        self.lat2_value = StringVar()
        self.lat2_value.set("")
        self.lat2 = tk.Label(self.detail_frame, textvariable=self.lat2_value)
        self.lat2.grid(row=0, column=4, padx=5, pady=5, sticky="w")
        self.lat2.config(width=40)
        
        #Data 1.2
        self.lon2_label = tk.Label(self.detail_frame, text="Lon 2:")
        self.lon2_label.grid(row=1, column=3, padx=5, pady=5, sticky="w")
        self.lon2_label.config(background="white", font="Arial 10 bold")
        
        self.lon2_value = StringVar()
        self.lon2_value.set("")
        self.lon2 = tk.Label(self.detail_frame, textvariable=self.lon2_value)
        self.lon2.grid(row=1, column=4, padx=5, pady=5, sticky="w")
        self.lon2.config(width=40)
        
        #Data 1.3
        self.latSet_label = tk.Label(self.detail_frame, text="Lat Value:")
        self.latSet_label.grid(row=2, column=3, padx=5, pady=5, sticky="w")
        self.latSet_label.config(background="white", font="Arial 10 bold")
        
        self.latSet = tk.Entry(self.detail_frame)
        self.latSet.grid(row=2, column=4, padx=5, pady=5, sticky="w")
        self.latSet.config(background="#dcdcdc", justify="left")
        
        #Data 1.4
        self.longSet_label = tk.Label(self.detail_frame, text="Long Value:")
        self.longSet_label.grid(row=3, column=3, padx=5, pady=5, sticky="w")
        self.longSet_label.config(background="white", font="Arial 10 bold")
        
        self.longSet = tk.Entry(self.detail_frame)
        self.longSet.grid(row=3, column=4, padx=5, pady=5, sticky="w")
        self.longSet.config(background="#dcdcdc", justify="left")
        
        #Data 1.5
        self.idxLabel = tk.Label(self.detail_frame, text="Pos:")
        self.idxLabel.grid(row=4, column=3, padx=5, pady=5, sticky="w")
        self.idxLabel.config(background="white", font="Arial 10 bold")
        
        self.clicked = StringVar() 
        self.option = [
            "1","2","3","4","5","6"
        ]
        # initial menu text 
        self.clicked.set( self.option[0] ) 
        self.index = tk.OptionMenu(self.detail_frame,self.clicked, *self.option )
        self.index.grid(row=4, column=4, padx=5, pady=5, sticky="w")
        
        self.save_coor = tk.Button(self.detail_frame, text="  Simpan  ", command=self.updateLatLon)
        self.save_coor.grid(row=5, column=4, padx=5, pady=5, sticky="w")
        
        self.save_coor = tk.Button(self.detail_frame, text="  Simpan & Next  ", command=self.saveLatLon)
        self.save_coor.grid(row=5, column=5, padx=5, pady=5, sticky="w")
        
        
        # Baris kedua, kolom kedua untuk tombol pengaturan counter dan posisi
        self.control_frame = tk.Frame(self)
        self.control_frame.grid(row=1, column=2, padx=10, pady=10, sticky="nsew", rowspan=2)
        
        self.update_counter_button = tk.Button(self.control_frame, text="Counter Plus", command=self.update_counter_plus)
        self.update_counter_button.grid(row=8, column=0, padx=5, pady=5, sticky="w", columnspan=2)
        
        self.update_position_button = tk.Button(self.control_frame, text="Counter Min", command=self.update_counter_min)
        self.update_position_button.grid(row=8, column=0, padx=5, pady=5, sticky="e", columnspan = 2)
        
        self.inverse = tk.Button(self.control_frame, text="Inverse", command=self.inverse)
        self.inverse.grid(row=8, column=3, padx=5, pady=5, sticky="w")
        
        self.upspeed = tk.Button(self.control_frame, text="Speed Up", command=self.speedUp)
        self.upspeed.grid(row=9, column=0, padx=5, pady=5, sticky="w")
        
        self.downspeed = tk.Button(self.control_frame, text="Speed Down", command=self.speedDown)
        self.downspeed.grid(row=9, column=0, padx=5, pady=5, sticky="e")
        
        self.motor_set = tk.Button(self.control_frame, text="Motor Set", command=self.setMotor)
        self.motor_set.grid(row=10, column=0, padx=5, pady=5, sticky="w")
        
        self.reset_round = tk.Button(self.control_frame, text="Reset", command=self.resetRound)
        self.reset_round.grid(row=10, column=0, padx=5, pady=5, sticky="e")
        
        #Informasi lat dan long
        self.infLabel = tk.Label(self.control_frame, text="Nilai Lat & Long:")
        self.infLabel.grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.infLabel.config(font="Ubuntu 14 bold")
        
        #LatLon1
        self.lat2Label = tk.Label(self.control_frame, text="1 ) Lat & Lon :")
        self.lat2Label.grid(row=3, column=0, padx=5, pady=5, sticky="w")
        
        self.lat1Value = StringVar()
        self.lat1Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat1Value)
        self.infLabel.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        
        #LatLon2
        self.lat2Label = tk.Label(self.control_frame, text="2 ) Lat & Lon :")
        self.lat2Label.grid(row=4, column=0, padx=5, pady=5, sticky="w")
        
        self.lat2Value = StringVar()
        self.lat2Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat2Value)
        self.infLabel.grid(row=4, column=1, padx=5, pady=5, sticky="w")
        
        #LatLon3
        self.lat3Label = tk.Label(self.control_frame, text="3 ) Lat & Lon :")
        self.lat3Label.grid(row=5, column=0, padx=5, pady=5, sticky="w")
        
        self.lat3Value = StringVar()
        self.lat3Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat3Value)
        self.infLabel.grid(row=5, column=1, padx=5, pady=5, sticky="w")
        
        #LatLon4
        self.lat4Label = tk.Label(self.control_frame, text="4 ) Lat & Lon :")
        self.lat4Label.grid(row=3, column=3, padx=5, pady=5, sticky="w")
        
        self.lat4Value = StringVar()
        self.lat4Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat4Value)
        self.infLabel.grid(row=3, column=4, padx=5, pady=5, sticky="w")
        
        #LatLon5
        self.lat5Label = tk.Label(self.control_frame, text="5 ) Lat & Lon :")
        self.lat5Label.grid(row=4, column=3, padx=5, pady=5, sticky="w")
        
        self.lat5Value = StringVar()
        self.lat5Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat5Value)
        self.infLabel.grid(row=4, column=4, padx=5, pady=5, sticky="w")
        
        #LatLon6
        self.lat6Label = tk.Label(self.control_frame, text="6 ) Lat & Lon :")
        self.lat6Label.grid(row=5, column=3, padx=6, pady=5, sticky="w")
        self.lat6Value = StringVar()
        self.lat6Value.set("0.0")
        self.infLabel = tk.Label(self.control_frame, textvariable=self.lat6Value)
        self.infLabel.grid(row=5, column=4, padx=5, pady=5, sticky="w")
        
        #Barier
        self.b1 = tk.Label(self.control_frame, text=" | ")
        self.b1.grid(row=3, column=2, padx=5, pady=5, sticky="w")
        self.b2 = tk.Label(self.control_frame, text=" | ")
        self.b2.grid(row=4, column=2, padx=5, pady=5, sticky="w")
        self.b3 = tk.Label(self.control_frame, text=" | ")
        self.b3.grid(row=5, column=2, padx=5, pady=5, sticky="w")
    
        #Informasi arah kapal
        self.arahLabel = tk.Label(self.control_frame, text="Arah kapal dri AI:")
        self.arahLabel.grid(row=6, column=0, padx=5, pady=5, sticky="w")
        self.arahLabel.config(font="Ubuntu 14 bold")
    
        self.arahValue = StringVar()
        self.arahValue.set("[ 0 | 0 ]")
        self.arahField = tk.Label(self.control_frame, textvariable=self.arahValue)
        self.arahField.grid(row=7, column=0, padx=5, pady=5, sticky="w")
        
        self.invValue = StringVar()
        self.invValue.set("[ 0 | 0 ]")
        self.invField = tk.Label(self.control_frame, textvariable=self.invValue)
        self.invField.grid(row=7, column=3, padx=5, pady=5, sticky="w")
        
        #tambahan
        self.detail_frames = tk.Frame(self, bg="white", width=10, height=10)
        self.detail_frames.grid(row=2, column=0,columnspan=2, padx=10, pady=10, sticky="nsew")
        self.log_label = tk.Label(self.detail_frames, text="Log:")
        self.log_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.log_label.config(background="white", font="Arial 12 bold")
        
        self.log_value = StringVar()
        self.log_value.set("0")
        self.log_area = tk.Label(self.detail_frames, textvariable=self.log_value)
        self.log_area.grid(row=1, column=0, padx=5, pady=5, sticky="w")
        
        self.log_res = StringVar()
        self.log_res.set("0")
        self.log_res_value = tk.Label(self.detail_frames, textvariable=self.log_res)
        self.log_res_value.grid(row=2, column=0, padx=5, pady=5, sticky="w")
        
        # Counter dan posisi default
        self.timer = 3
        self.counter = 0
        self.x_position = 0
        self.y_position = 0
        self.motor = 0
        self.curIdx = 0
        self.lll = []
        
        # Simulasi video update
        self.after(1000, self.update_video_frames)
        
        mqtt.setForm(self)
        self.mqtt = mqtt
    def speedUp(self):
        if self.mqtt is not None : 
            self.mqtt.speed += 10
            
    def setTimeOut(self):
        self.timer = 3
    
    def resetRound(self):
        self.timer -= 1
        tmr = Timer(3,self.setTimeOut, None)
        tmr.start()
        if self.mqtt is not None and self.timer <= 0: 
            self.mqtt.reset()
            self.mqtt.loadSetPoint()
            
    def saveLatLon(self):
        if self.mqtt is not None or True:
            self.mqtt.Lats.append(self.mqtt.lat)
            self.mqtt.Lons.append(self.mqtt.lon)
            self.lll.append(self.curIdx)
            with open("coor.json", "w+") as fil:
                fil.write(json.dumps({
                    "lats" : self.mqtt.Lats,
                    "lons" : self.mqtt.Lons
                }))
            self.curIdx += 1
            self.option = [str(idx) for idx in range(self.curIdx)]
            self.index = tk.OptionMenu(self.detail_frame,self.clicked, *self.option )
            self.index.grid(row=4, column=4, padx=5, pady=5, sticky="w")

            
    def speedDown(self):
        if self.mqtt is not None:
            self.mqtt.speed -= 10
            
    def inverse(self):
        if self.mqtt is not None:
            self.mqtt.inv = 0 if self.mqtt.inv == 1 else 1
            self.invValue.set(str(self.mqtt.inv))
            
        
    def updateLatLon(self):
        if self.mqtt is not None or True:
            self.mqtt.Lats[self.clicked.get] = self.mqtt.lat
            self.mqtt.Lons[self.clicked.get] = self.mqtt.lon
            with open("coor.json", "w+") as fil:
                fil.write(json.dumps({
                    "lats" : self.lll,
                    "lons" : self.lll
                }))

    def update_video_frames(self):
        # Simulasi update frame video setiap detik
        current_time = time.strftime('%H:%M:%S')
        self.video_frame_1.config(text=f"Video 1 - {current_time}")
        self.video_frame_2.config(text=f"Video 2 - {current_time}")
        self.video_frame_3.config(text=f"Video 3 - {current_time}")
        self.after(1000, self.update_video_frames)  # Update setiap 1 detik
    
    def display_frame(self, label, frame):
        # Konversi frame BGR OpenCV ke RGB dan menampilkan di Tkinter Label
        frame = cv2.resize(frame , (400, 300))
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        label.config(image=imgtk, width=400, height = 300)
        label.imgtk = imgtk
    
    def setMotor(self):
        self.mqtt.counter - 1
        # self.mqtt.sendPub()

        self.motor = 1 if self.motor == 0 else 0
        self.arahValue.set("Motor hidup" if self.motor ==1 else "Motor Mati")
        if self.mqtt is not None: self.mqtt.motor = self.motor
    
    def update_counter_plus(self):
        self.counter_value.set(str(self.mqtt.counter))
        if self.mqtt is not None:
            self.mqtt.mqttc.emit("setCounter",{"event":"sc","value" : self.mqtt.counter + 1})
        
    def update_counter_min(self):
        self.counter_value.set(str(self.mqtt.counter))
        if self.mqtt is not None:
            self.mqtt.mqttc.emit("setCounter",{"event":"sc","value" :self.mqtt.counter - 1 })

    def update_position(self):
        self.x_position += 10
        self.y_position += 5
        self.coordinate_value.set(f"(X: {self.x_position}, Y: {self.y_position})")

    def update_log(self, msg):
        self.log_value.set(str(msg))

        
    def on_close(self):
        # Fungsi untuk menutup aplikasi dengan benar dan melepaskan resource
        self.active = False
        self.destroy()

    def onKeyPress(self,event):
        print("You pressed ", event.char)
        if event.char.lower() == "a":
            print("Kapal ke kiri")
            if self.mqtt is not None : self.mqtt.mqttc.emit("belok",{"event" : "belok","angle" : -10})
        if event.char.lower() == "s":
            if self.mqtt is not None : self.mqtt.speed = 1350
            print("Kapal mundur")
        if event.char.lower() == "w":
            print("Kapal maju")
            if self.mqtt is not None : self.mqtt.speed = 1700
        if event.char.lower() == "d":
            print("Kapal kekanan")
            if self.mqtt is not None : self.mqtt.mqttc.emit("belok",{"event" : "belok","angle" : 10})
        if event.char.lower() == "x":
            self.mqtt.loadConf()
            print("Load Konf")
        if event.char.lower() == " ":
            print("Kapal stop")
            if self.mqtt is not None : self.mqtt.speed = 1550

def launchApp(mqtt):
    global app
    app = VideoMonitorApp(mqtt)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.bind('<KeyPress>', app.onKeyPress)
    app.mainloop()

# launchApp(None)

