
import argparse
import csv
import os
import platform
import sys
from pathlib import Path
import torch
import requests
import cv2
import numpy as np
from threading import Thread
import mqtt_test
import json

import form 
import time
import socketio
from utils.folder import create_folder_in_public, save_image_to_folder, get_image_path
import asyncio
import pathlib
from datetime import datetime
from flask_cors import CORS, cross_origin
from flask_socketio import SocketIO, send
from flask import Flask, Request, Response, abort, jsonify, send_file
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

sio = socketio.AsyncClient()
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading',  max_http_buffer_size=100000000,  # 100 MB buffer size
                    ping_interval=60,                # 60 detik interval antara ping
                    ping_timeout=120,                # 120 detik timeout sebelum disconnect
                    allow_upgrades=True,             # Mengizinkan upgrade protokol
                    engineio_logger=True)

mqtt_test.mqtt(socketio)
dataKapal = {}
date = datetime.today().strftime('%d/%m/%Y')
times = datetime.today().strftime('%H:%M:%S')
days = [ "Mon", "Thus", "Wed", "Thurs", "Fri", "Sat","Sun"]
day = days[datetime.today().weekday()%len(days)]

new_path = create_folder_in_public()
img_id = 0

@socketio.on('message')
def handle_message(message):
    print('Message from client:', message)
    #send(f"Echo: {message}", broadcast=True)
    
@socketio.on('data')
def data(data):
    mqtt_test.mymqtt.on_message(data)
    
@socketio.on("join")
def join(data):
    mqtt_test.mymqtt.sendPub()
    
@app.route('/data')
def data():
    data = {}
    if mqtt_test.mymqtt is not None:
        data = {
            "hdg" : mqtt_test.mymqtt.azimuth,
            "sog" : mqtt_test.mymqtt.sog,
            "speedKm" : mqtt_test.mymqtt.speedKm,
            "cog" : mqtt_test.mymqtt.cog,
            "day" : day,
            "date" : date,
            "gps" : {
                "lat" : mqtt_test.mymqtt.lat,
                "latDir" : mqtt_test.mymqtt.latDir,
                "long"  : mqtt_test.mymqtt.lon,
                "lonDir" : mqtt_test.mymqtt.lonDir,
                "centerLat" : 0,
                "centerLong" : 0
            },
            "position" : mqtt_test.mymqtt.getPosition(),
            "progress" : f'{mqtt_test.mymqtt.getProgress()}%',
            "lintasan" :  mqtt_test.mymqtt.lintasan,
            "counter" : mqtt_test.mymqtt.counter,
            "coor" : [mqtt_test.mymqtt.lats, mqtt_test.mymqtt.lons],
            # "gps" : f"{mqtt_test.mymqtt.latDir} {mqtt_test.mymqtt.lat} {mqtt_test.mymqtt.lonDir} {mqtt_test.mymqtt.lon}"
        }
    return jsonify(data)
    
frame1 = [None, None, None, None]


def generate_frames(idx):
    global frame1
    while True:
        if frame1[idx] is not None:
            # Mengonversi frame dari BGR (OpenCV) ke format JPEG
            ret, buffer = cv2.imencode('.jpg', frame1[idx])
            frame = buffer.tobytes()

            # Mengirim frame dengan format multipart
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    


@app.route('/cam1')
def cam1():
    return Response(generate_frames(1),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam2')
def cam2():
    return Response(generate_frames(2),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
@app.route('/cam3')
def cam3():
    return Response(generate_frames(3),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_image_len')
def get_image_len():
    return jsonify({"len":img_id})

@app.route('/get_image/<int:image_id>', methods=['GET'])
def get_image(image_id):
    image_path = get_image_path(new_path, image_id)
    
    if image_path:
        # Kirim gambar menggunakan send_file
        return send_file(image_path, mimetype='image/jpeg')
    else:
        # Jika file tidak ditemukan, kembalikan 404
        abort(404, description="File tidak ditemukan")

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.torch_utils import select_device, smart_inference_mode

def stream_video(url, window_name):
    stream = requests.get(url, stream=True)
    bytes_data = bytes()

    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) == ord('q'):
                    break
                
    cv2.destroyAllWindows()

weights="./best.pt"  # model path or triton URL
data="./datasets/box/data.yaml" # dataset.yaml pat

device = select_device("")
model = DetectMultiBackend(weights=weights, device=device, dnn=False, data=data, fp16=False)
    
@smart_inference_mode()
async def run(
    idx,
    camsync = False,
    mode = 0,
    weights=ROOT / "./best.pt",  # model path or triton URL
    source="http://192.168.2.148:8081/?action=stream",  # file/dir/URL/glob/screen/0(webcam)
    data=ROOT / "datasets/box/data.yaml",  # dataset.yaml path
    imgsz=(640, 640),  # inference size (height, width)
    conf_thres=0.5,  # confidence threshold
    iou_thres=0.45,  # NMS IOU threshold
    max_det=1000,  # maximum detections per image
    device="",  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    view_img=False,  # show results
    save_txt=False,  # save results to *.txt
    save_csv=False,  # save results in CSV format
    save_conf=False,  # save confidences in --save-txt labels
    save_crop=False,  # save cropped prediction boxes
    nosave=False,  # do not save images/videos
    classes=None,  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False,  # class-agnostic NMS
    augment=False,  # augmented inference
    visualize=False,  # visualize features
    update=False,  # update all models
    project=ROOT / "runs/detect",  # save results to project/name
    name="exp",  # save results to project/name
    exist_ok=False,  # existing project/name ok, do not increment
    line_thickness=3,  # bounding box thickness (pixels)
    hide_labels=False,  # hide labels
    hide_conf=False,  # hide confidences
    half=False,  # use FP16 half-precision inference
    dnn=False,  # use OpenCV DNN for ONNX inference
    vid_stride=1,  # video frame-rate stride
    useAI = True
):
    global model, frame1, img_id
    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / "labels" if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir
    # Load model
   
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader

    bs = 1  # batch_size
    curTime = time.time()
    boxTime = time.time()
 
    view_img = check_imshow(warn=True)
    dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride, form=form)
    bs = len(dataset)

    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
    prevAngle = 0

    for path, im, im0s, vid_cap, s in dataset: 
        

        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            if model.xml and im.shape[0] > 1:
                ims = torch.chunk(im, im.shape[0], 0)

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            if model.xml and im.shape[0] > 1:
                pred = None
                for image in ims:
                    if pred is None:
                        pred = model(image, augment=augment, visualize=visualize).unsqueeze(0)
                    else:
                        pred = torch.cat((pred, model(image, augment=augment, visualize=visualize).unsqueeze(0)), dim=0)
                pred = [pred, None]
            else:
                pred = model(im, augment=augment, visualize=visualize)
        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
	
          

        # Second-stage classifier (optional) hjkhkjhkjh
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)
        for i, det in enumerate(pred):  # per image
            seen += 1
            save_box = False
            p, im0, frame = path[i], im0s[i].copy(), dataset.count
            frame1[idx] = im0s[i]
            s += f"{i}: "
            box = (0,0), (0,0)
            ada = False
            targetAngle = 0

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / "labels" / p.stem) + ("" if dataset.mode == "image" else f"_{frame}")  # im.txt
            s += "%gx%g " % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            pilot = "MANUAL"
            sizeRed = 0
            sizeGreen = 0
            size = {"green_buoy" : 0, "red_buoy" : 0}
            inv = 0
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                prevX = 0
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = names[c] if hide_conf else f"{names[c]}"
                    confidence = float(conf)
                    confidence_str = f"{confidence:.2f}"

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(f"{txt_path}.txt", "a") as f:
                            f.write(("%g " * len(line)).rstrip() % line + "\n")

                    if view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f"{names[c]} {conf:.2f}")
                        pos = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                        if int(pos[0][1]) > 180 and int(pos[1][1]) < 420 : 
                            pass
                        if mode == 0 and (names[c] == "green_buoy" or names[c]=="red_buoy") :annotator.box_label(xyxy, label, color=colors(c, True))
                        if mode == 1 and (names[c] == "blue_box" or names[c]=="green_box") :
                            annotator.box_label(xyxy, label, color=colors(c, True))
                            save_box = True
                    # if save_crop:
                        if names[c] == "blue_box" or names[c] == "green_box" : save_one_box(xyxy, imc, file=save_dir / "crops" / names[c] / f"{p.stem}.jpg", BGR=True)
                    box = xyxy
                    
                    inValue = mqtt_test.mymqtt.inv
                       
                    if pilot == "MANUAL" : 
                        pilot = "AUTO" if (abs(int(box[0]) - int(box[2])) > 25) else "MANUAL"
                    if mode == 0 and names[c] == ("green_buoy" if  inValue == 0 else "red_buoy") and pilot == "AUTO":
                        if int(box[2] ) >= 175 and abs(int(box[0]) - int(box[2])) > size[("green_buoy" if  inValue == 0 else "red_buoy")] : 
                           size[("green_buoy" if  inValue == 0 else "red_buoy")] = abs(int(box[0]) - int(box[2]))
                           prevX = max(0, int(box[2] ) - 175)
                           targetAngle = -45 if (abs(int(box[0]) - int(box[2])) > 50) else -15
                    if mode == 0 and names[c] == ("green_buoy" if  inValue == 1 else "red_buoy") and pilot == "AUTO":
                        if int(box[0] ) <= 465 and abs(abs(int(box[0]) - int(box[2]))) > size[("green_buoy" if  inValue == 1 else "red_buoy")]: 
                           size[("green_buoy" if  inValue == 1 else "red_buoy")] = abs(int(box[0]) - int(box[2]))
                           targetAngle = (45 if (abs(int(box[0]) - int(box[2])) > 50) else 15) if max(465-int(box[0]), 0) > prevX else targetAngle
                    print( (int(box[0]), int(box[1])), (int(box[2]), int(box[3])))
            
            if prevAngle is not targetAngle and mode==0 and abs(curTime - time.time()) > 0.1 and useAI:
                curTime = time.time() 
                mqtt_test.mymqtt.mqttc.emit("addAngle",{
                    "event" : "ang",
                    "angle" : targetAngle
                })
            prevAngle = targetAngle
            if save_box and abs(time.time() - boxTime) > 1 and mqtt_test.mymqtt.counter >= mqtt_test.mymqtt.captureCounter:
                img_id += 1
                boxTime = time.time()
                save_image_to_folder(410,frame1[idx], new_path, img_id, f"Lat & Lon : {mqtt_test.mymqtt.lat} | {mqtt_test.mymqtt.lon} ")
                save_image_to_folder(440,frame1[idx], new_path, img_id, f"COG & SOG : {mqtt_test.mymqtt.cog}° | {mqtt_test.mymqtt.sog}kn")
                save_image_to_folder(470,frame1[idx], new_path, img_id, f"Time : {day} {date} {times}")
                if camsync:
                    save_image_to_folder(410,frame1[1], new_path, img_id + " 2", f"Lat & Lon : {mqtt_test.mymqtt.lat} | {mqtt_test.mymqtt.lon} ")
                    save_image_to_folder(440,frame1[1], new_path, img_id + " 2", f"COG & SOG : {mqtt_test.mymqtt.cog}° | {mqtt_test.mymqtt.sog}kn")
                    save_image_to_folder(470,frame1[1], new_path, img_id + " 2", f"Time : {day} {date} {times}")
    
            # Stream results
            im0 = annotator.result()
            frames = None
            if view_img:
                if idx == 1:
                    frames = form.app.video_frame_1
                elif idx == 2:
                    frames = form.app.video_frame_2
                elif idx == 3:
                    frames = form.app.video_frame_3
                if form.app is not None:
                    if mode == 0 :cv2.line(im0, (320, 0), (320, 479), (0, 255, 0), thickness=2)
                    if mode == 0 :cv2.line(im0, (0, 180), (640, 180), (0, 255, 0), thickness=2)
                    if mode == 0 :cv2.line(im0, (0, 400), (640, 400), (0, 255, 0), thickness=2)
                    if mode == 1 :cv2.line(im0, (0, 230), (640, 230), (255,0, 0), thickness=2)
                    if mode == 1 :cv2.line(im0, (310, 0), (310, 480), (255,0, 0), thickness=2)

                    if mode == 0 :cv2.putText(im0,pilot, (20,50), 0, 2, (255,0,0) , thickness=2, lineType=cv2.LINE_AA)

                    form.app.display_frame(frames,im0)

            # Save results (image with detections)
            if False:
                if dataset.mode == "image":
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix(".mp4"))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
                    vid_writer[i].write(im0)

        # Print time (inference-only)
        LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

    # Print results
    t = tuple(x.t / seen * 1e3 for x in dt)  # speeds per image
    LOGGER.info(f"Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}" % t)
    if True:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ""
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)

async def mains():
    return
    try:
        await sio.connect('http://localhost:3000')
        await sio.wait()
    except :
        print("Error ")
        
async def inference1():
    global form
    time.sleep(3)
    asyncio.create_task(run(idx=1, mode=1,source="http://192.168.1.5:8080/?action=stream"))
   
async def inference2():
    global form
    asyncio.create_task(run(idx=2,source="http://192.168.1.5:8081/?action=stream"))
    
async def inference3():
    global form
    time.sleep(6)
    asyncio.create_task(run(camsync=True,idx=3,mode=1,source="http://192.168.1.3:8080/?action=stream"))

def start1():
    asyncio.run(inference1())
    #run(idx=1, mode=1,source="http://192.168.1.5:8080/?action=stream")
def start2(): 
    asyncio.run(inference2())

    #run(idx=2,source="http://192.168.1.5:8081/?action=stream")
def start3():  
    asyncio.run(inference3())

    #run(idx=3,mode=1,source="http://192.168.1.4:8080/?action=stream")
    
def startServer():
    app.run(host='0.0.0.0', port=5000)

formThread = Thread(target=form.launchApp, args=(mqtt_test.mymqtt,))
inf1 = Thread(target=start1, daemon=True)
inf2 = Thread(target=start2)
inf3 = Thread(target=start3)
dataThread = Thread(target=mqtt_test.mymqtt.sendData)
# socketThread = Thread(target=mains)
serverThread = Thread(target=startServer)
# socketThread.start()
formThread.start()
inf1.start()
dataThread.start()
inf2.start()
inf3.start()
serverThread.start()
# socketThread.join()
formThread.join()
inf1.join()
dataThread.join()
inf2.join(0)
inf3.join(0)
serverThread.join()