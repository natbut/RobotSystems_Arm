#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import * # color_range from here
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red')
def setTargetColor(target_color):
    global __target_color

    print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  #计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  #只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max  #返回最大的轮廓

servo1 = 500
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True
def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global detect_color
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    __target_color = ()
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True

def init():
    print("ColorSorting Init")
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Stop")

def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorSorting Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False 
world_X, world_Y = 0, 0
def move():
    global rect
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    
    #放置坐标
    coordinate = {
        'red':   (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5,  1.5),
        'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
    }
    while True:
        if __isRunning:        
            if detect_color != 'None' and start_pick_up:  #如果检测到方块没有移动一段时间后，开始夹取
                #移到目标位置，高度6cm, 通过返回的结果判断是否能到达指定位置
                #如果不给出运行时间参数，则自动计算，并通过结果返回
                set_rgb(detect_color)
                setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2]/1000) #如果可以到达指定位置，则获取运行时间

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle) #计算夹持器需要旋转的角度
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # 爪子张开
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  #夹持器闭合
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  #机械臂抬起
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)   
                    time.sleep(result[2]/1000)
                    
                    if not __isRunning:
                        continue                   
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                    time.sleep(0.5)
                    
                    if not __isRunning:
                        continue                    
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # 爪子张开  ，放下物体
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                    time.sleep(0.8)

                    initMove()  # 回到初始位置
                    time.sleep(1.5)

                    detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    set_rgb(detect_color)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)
          
# Custom Perception Class 

class perception():
    
    def __init__(self):
        self.roi = (0,0,0,0)
        self.rect = None
        self.count = 0
        self.get_roi = True
        self.center_list = []
        self.start_pick_up = False
        self.rotation_angle = 0
        self.last_x = 0
        self.last_y = 0
        self.world_X = 0
        self.world_Y = 0
        self.start_count_t1 = True
        self.t1 = 0
        self.detect_color = None
        self.draw_color = range_rgb["black"]
        self.color_list = []
        
        self.img = None
        self.size = (640, 480)
        
        self.frame_lab = None
        self.distance = 0
    
    def process_img(self, img):
        self.img = img
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(self.img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    
        if self.get_roi and not start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)      
        self.frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    
    def identify_max_overall_area(self, i, color_area_max, max_area, areaMaxContour_max):
        frame_mask = cv2.inRange(self.frame_lab, color_range[i][0], color_range[i][1]) 
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
        areaMaxContour, area_max = getAreaMaxContour(contours) 
        if areaMaxContour is not None:
            if area_max > max_area:
                max_area = area_max
                color_area_max = i
                areaMaxContour_max = areaMaxContour
                
        return color_area_max, max_area, areaMaxContour_max
    
    def localize_object(self, color_area_max, areaMaxContour_max):
        self.rect = cv2.minAreaRect(areaMaxContour_max)
        box = np.int0(cv2.boxPoints(self.rect))
            
        self.roi = getROI(box)
        self.get_roi = True
        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, square_length)
             
        self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)
            
        cv2.drawContours(self.img, [box], -1, range_rgb[color_area_max], 2)
        cv2.putText(self.img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[color_area_max], 1)
            
        self.distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
        self.last_x, self.last_y = self.world_x, self.world_y
        
    def init_pickup(self, color_area_max):
        if color_area_max == 'red':
            color = 1
        elif color_area_max == 'green':
            color = 2
        elif color_area_max == 'blue':
            color = 3
        else:
            color = 0
        self.color_list.append(color)
                
        if self.distance < 0.5:
            self.count += 1
            self.center_list.extend((self.world_x, self.world_y))
            if self.start_count_t1:
                self.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1:
                self.rotation_angle = self.rect[2] 
                self.start_count_t1 = True
                self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                self.center_list = []
                self.count = 0
                #self.start_pick_up = True
        else:
            self.t1 = time.time()
            self.start_count_t1 = True
            self.center_list = []
            self.count = 0

        if len(color_list) == 3:
            color = int(round(np.mean(np.array(self.color_list))))
            self.color_list = []
            if color == 1:
                self.detect_color = 'red'
                self.draw_color = range_rgb["red"]
            elif color == 2:
                self.detect_color = 'green'
                self.draw_color = range_rgb["green"]
            elif color == 3:
                self.detect_color = 'blue'
                self.draw_color = range_rgb["blue"]
            else:
                self.detect_color = 'None'
                self.draw_color = range_rgb["black"]
                        
    def reset_draw_detect(self):
        self.draw_color = (0, 0, 0)
        self.detect_color = "None"
        
    def annotate_img(self):
        return cv2.putText(self.img, "Color: " + self.detect_color, (10, self.img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)


def identify_and_label_block(perception, img):
    global __isRunning

    if not __isRunning:
        return img
    
    perception.process_img(img)
    
    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    
    if not perception.start_pick_up:
        for i in color_range:
            if i in __target_color:
                color_area_max, max_area, areaMaxContour_max = perception.identify_max_overall_area(i, color_area_max, max_area, areaMaxContour_max)
                
        if max_area > 2500:
            perception.localize_object(color_area_max, areaMaxContour_max)
            
            if not perception.start_pick_up:
                perception.init_pickup(color_area_max)
                
        else:
            if not perception.start_pick_up:
                perception.reset_draw_detect()

    return perception.annotate_img()


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', 'green', 'blue')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    p = perception()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = identify_and_label_block(p, frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()

