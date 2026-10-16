from time import sleep
from maix import camera, display, image, time, uart, app ,nn
from struct import pack
from maix._maix.image import Threshold
import struct

def send_data(binary_data: bytes):
    serial.write(binary_data)  # 直接发送字节

def on_received(serial : uart.UART, data : bytes): 
    global find_which
    if len(data) == 4:
        if data[0] == 0xaa and data[1] == 0xbb and data[3] == 0xcc:
            find_which = data[2]

def find_qr(qrcodes):
    global find_which
    kind_find = ""
    id_find = ""
    ref_find = "none"

    if find_which == 1:
        kind_find = "item"
        id_find = "A"
        ref_find = "1"
    elif find_which == 2:
        kind_find = "item"
        id_find = "B"
        ref_find = "1"
    elif find_which == 3:
        kind_find = "item"
        id_find = "C"
        ref_find = "1"

    elif find_which == 4:
        kind_find = "item"
        id_find = "D"
        ref_find = "1"

    elif find_which == 5:
        kind_find = "place"
        id_find = "A"

    elif find_which == 6:
        kind_find = "place"
        id_find = "B"

    elif find_which == 7:
        kind_find = "place"
        id_find = "C"
    
    elif find_which == 8:
        kind_find = "place"
        id_find = "D"

    elif find_which == 9:
        kind_find = "place"
        id_find = "a"

    elif find_which == 10:
        kind_find = "place"
        id_find = "b"

    elif find_which == 11:
        kind_find = "place"
        id_find = "c"
    
    elif find_which == 12:
        kind_find = "place"
        id_find = "d"

    elif find_which == 13:
        kind_find = "place"
        id_find = "e"

    elif find_which == 14:
        kind_find = "place"
        id_find = "f"
    
    elif find_which == 15:
        kind_find = "place"
        id_find = "abc"

    elif find_which == 16:
        kind_find = "place"
        id_find = "def"

    elif find_which == 17:
        kind_find = "place"
        id_find = "home"

    for qr in qrcodes:
        kind = qr.payload().split(':')[0]
        if kind == kind_find:
            body = qr.payload().split(':')[1]
            if kind_find == "place" and body == id_find:
                return qr.x(),qr.y()
            if kind_find == "item":
                id,ref = body.split(',')
                if  id == id_find and ref == ref_find:
                    return qr.x(),qr.y()
    return 0,0     

def data_pack():
    global find_which,x,y
    head = bytes([0xaa,0xbb,find_which])
    body= bytes([x >> 8 & 0xFF, (x & 0xFF),((y >> 8) & 0xFF),(y & 0xFF)])
    ass = bytes([0xcc])
    strr = head + body + ass 
    return strr

def data_pack_yolo(class_id,length):
    global find_which,x,y
    head = bytes([0xaa,0xbb,find_which])
    body = bytes([class_id,length])
    ass = bytes([0xcc])
    strr = head + body + ass 
    return strr

#-------------------------------------------------------主函数-----------------------------------------------------#
device = "/dev/ttyS0"
serial = uart.UART(device, 115200)

serial.set_received_callback(on_received)  # 设置串口回调函数

disp = display.Display()        # 屏幕初始化
cam = camera.Camera(320, 240)   # 摄像头初始化
cam.skip_frames(30)             # 跳过起始的30帧图像

detector = nn.YOLO11(model="/root/models/yolo11n_seg.mud")

# 全局变量声明及初始化
x = 0
y = 0
find_which = 0

send_data(bytes([0xaa,0xbb,0xcc]))
while not app.need_exit():
    img = cam.read()
    if find_which > 0 and find_which <= 17:
        qrcodes = img.find_qrcodes()
        x,y = find_qr(qrcodes)
        # x,y = find_qr(qrcodes,"item","apple","1")
        if x!=0 and y!=0:
            send_data(data_pack())
            img.draw_string(0, 0,f"({x},{y})", image.COLOR_RED)
    elif find_which == 18:
        objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)
        match_id = 0
        match_score = 0.00
        if len(objs) > 0:
            for obj in objs:
                # 显示识别到的是什么
                img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
                msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
                img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)
                if obj.score > match_score:
                    match_score = obj.score
                    match_id = obj.class_id
            if match_score > 0.55:
                send_data(data_pack_yolo(match_id,len(objs)))
    disp.show(img) 
        
    