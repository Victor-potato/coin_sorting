from tkinter import *
import cv2
import RPi.GPIO as GPIO
import time
import threading


GPIO.setmode(GPIO.BOARD)

stepper1_pins = [13,11,15,12]
stepper2_pins = [36,31,37,29]
feeder_pin = 16
sorter_pin = 32

class Conveyor:
    def __init__(self, pins):
        self.pins = pins
        self.flag = False
        self.exit = False
        self.seq = [
          [1,0,0,0],
          [1,1,0,0],
          [0,1,0,0],
          [0,1,1,0],
          [0,0,1,0],
          [0,0,1,1],
          [0,0,0,1],
          [1,0,0,1]
        ]
        for pin in self.pins:
            GPIO.setup(pin,GPIO.OUT)
            GPIO.output(pin,0)      
            
    def run1(self):
        while True:
            if self.exit:
                break
            if not self.flag:
                for pin in self.pins:
                    GPIO.output(pin,0)
                time.sleep(0.01)
            else:
                for i in self.seq:
                    for pin in range(4):
                        GPIO.output(self.pins[pin], i[pin])
                    time.sleep(0.01)
    def run2(self):
        while True:
            if self.exit:
                break
            if not self.flag:
                for pin in self.pins:
                    GPIO.output(pin,0)
                time.sleep(0.01)
            else:
                for i in self.seq[::-1]:
                    for pin in range(4):
                        GPIO.output(self.pins[pin], i[pin])
                    time.sleep(0.01)
    
    def start(self):
        self.flag = True
        
    def stop(self):
        self.flag = False
        
    def finish(self):
        self.exit = True

class Feeder:
    def __init__(self, pin):
        GPIO.setup(pin,GPIO.OUT)
        self.feeder = GPIO.PWM(pin,50)
        self.feeder.start(0)
        
    def start(self):
        self.feeder.ChangeDutyCycle(7.3)
        
    def stop(self):
        self.feeder.ChangeDutyCycle(0)
        
class Sorter:
    def __init__(self, pin):
        GPIO.setup(pin,GPIO.OUT)
        self.sorter = GPIO.PWM(pin,50)
        self.sorter.start(0)
        self.flag = False
        
    def special(self):
        self.sorter.ChangeDutyCycle(2+(120/18))
        time.sleep(0.5)
        self.sorter.ChangeDutyCycle(0)
        self.flag = True
        
    def normal(self):
        self.sorter.ChangeDutyCycle(2+(60/18))
        time.sleep(0.5)
        self.sorter.ChangeDutyCycle(0)
        self.flag = True
        
    def get_flag(self):
        return self.flag
        
    def reset_flag(self):
        self.flag = False



class GUI(Frame):
    def __init__(self, master, title, size, conveyor1, conveyor2, feeder, sorter):
        master.title(title)
        master.geometry(size)
        Frame.__init__(self, master)
        self.pack()
        self.conveyor1 = conveyor1
        self.conveyor2 = conveyor2
        self.feeder = feeder
        self.sorter = sorter
        self.exit = False
        self.coins = 0
        self.path1 = "/home/pi/Documents/thesis/Pictures1/"
        self.path2 = "/home/pi/Documents/thesis/Pictures2/"
        self.video = False
        self.coin_label = Label(self)
        self.coin_label["text"] = "Click START button to start"
        self.coin_label.pack()
        self.create_buttons()
    
    def start(self):
        self.exit = False
        self.t1 = threading.Thread(target=self.find_coin1)
        self.t2 = threading.Thread(target=self.find_coin2)
        self.t1.setDaemon(True)
        self.t2.setDaemon(True)
        self.t1.start()
        self.t2.start()
        
    def finish(self):
        self.exit = True
        
    def video_on(self):
        self.video = True
        
    def video_off(self):
        self.video = False
        cv2.destroyAllWindows()
    
    def clean_coins(self):
        self.coins = 0
        self.coin_label.config(text=str(self.coins)+" coins passed")
    
    def detect_circle(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (15, 15), 0)
        thresh = cv2.adaptiveThreshold(gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 1)
        cont_img = thresh.copy()
        contours, hierarchy = cv2.findContours(cont_img, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 12000:
                continue
            ellipse = cv2.fitEllipse(cnt)
            cv2.ellipse(img, ellipse, (0,255,0), 2)
            return True
        return False
    
    def find_coin1(self):
        cap1 = cv2.VideoCapture(0)
        count = 0
        state = "Waiting for Coin"
        self.conveyor1.start()
        self.conveyor2.start()
        self.feeder.start()
        self.sorter.normal()
        while cap1.isOpened():
            if self.exit:
                break
            (ref, frame) = cap1.read()
            rof = frame.copy()[60:470, 0:600]
            res = self.detect_circle(rof)
            if state == "Waiting for Coin":
                if res:
                    count += 1
                    if count > 5:
                        state = "Coin under Cam"
                        cv2.imwrite(self.path1 + "coin-" + str(self.coins) + '-' +
                                    time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", rof)
                        self.feeder.stop()
                        count = 0
                else:
                    count = 0
            elif state == "Coin under Cam":
                if not res:
                    count += 1
                    if count > 5:
                        #self.coins += 1
                        state = "Waiting for Coin"
                        #self.feeder.start()
                        count = 0
                else:
                    count = 0
            #self.coin_label.config(text=state+' '+str(self.coins))
            if self.video:
                cv2.imshow('Cam 1', rof)
            cv2.waitKey(10)
            
        cap1.release()
        cv2.destroyAllWindows()
        self.conveyor1.stop()
        self.conveyor2.stop()
        self.feeder.stop()
        self.coin_label.config(text=str(self.coins)+" coins passed")
        
    def find_coin2(self):
        cap2 = cv2.VideoCapture(1)
        count = 0
        state = "Waiting for Coin"
        while cap2.isOpened():
            if self.exit:
                break
            (ref, frame) = cap2.read()
            rof = frame.copy()
            res = self.detect_circle(rof)
            if state == "Waiting for Coin":
                if res:
                    count += 1
                    if count > 5:
                        state = "Coin under Cam"
                        cv2.imwrite(self.path2 + "coin-" + str(self.coins) + '-' +
                                    time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", rof)
                        self.feeder.stop()
                        self.conveyor1.stop()
                        self.conveyor2.stop()
                        count = 0
                else:
                    count = 0
            elif state == "Coin under Cam":
                if self.sorter.get_flag():
                    self.conveyor2.start()
                    if not res:
                        count += 1
                        if count > 5:
                            self.coins += 1
                            state = "Waiting for Coin"
                            self.feeder.start()
                            self.conveyor1.start()
                            self.sorter.reset_flag()
                            count = 0
                    else:
                        count = 0

            self.coin_label.config(text=state+' '+str(self.coins))
            if self.video:
                cv2.imshow('Cam 2', rof)
            cv2.waitKey(10)
            
        cap2.release()
        cv2.destroyAllWindows()
    
    def create_buttons(self):
        self.on = Button(self)
        self.on["text"] = "START"
        self.on["command"] =  self.start
        self.on.pack()
        
        self.off = Button(self)
        self.off["text"] = "FINISH"
        self.off["command"] =  self.finish
        self.off.pack()
        
        self.videoOn = Button(self)
        self.videoOn["text"] = "Video On"
        self.videoOn["command"] =  self.video_on
        self.videoOn.pack()
        
        self.videoOff = Button(self)
        self.videoOff["text"] = "Video Off"
        self.videoOff["command"] =  self.video_off
        self.videoOff.pack()
        
        self.feeder_start = Button(self)
        self.feeder_start["text"] = "Feeder START"
        self.feeder_start["command"] =  self.feeder.start
        self.feeder_start.pack()
        
        self.feeder_stop = Button(self)
        self.feeder_stop["text"] = "Feeder STOP"
        self.feeder_stop["command"] =  self.feeder.stop
        self.feeder_stop.pack()
        
        self.sorter_special = Button(self)
        self.sorter_special["text"] = "Sorter SPECIAL"
        self.sorter_special["command"] =  self.sorter.special
        self.sorter_special.pack()
        
        self.sorter_normal = Button(self)
        self.sorter_normal["text"] = "Sorter NORMAL"
        self.sorter_normal["command"] =  self.sorter.normal
        self.sorter_normal.pack()
        
        self.conv1_start = Button(self)
        self.conv1_start["text"] = "Conveyor1 START"
        self.conv1_start["command"] =  self.conveyor1.start
        self.conv1_start.pack()

        self.conv1_stop = Button(self)
        self.conv1_stop["text"] = "Conveyor1 STOP"
        self.conv1_stop["command"] = self.conveyor1.stop
        self.conv1_stop.pack()
        
        self.conv2_start = Button(self)
        self.conv2_start["text"] = "Conveyor2 START"
        self.conv2_start["command"] =  self.conveyor2.start
        self.conv2_start.pack()

        self.conv2_stop = Button(self)
        self.conv2_stop["text"] = "Conveyor2 STOP"
        self.conv2_stop["command"] = self.conveyor2.stop
        self.conv2_stop.pack()
        
        self.clean = Button(self)
        self.clean["text"] = "CLEAN"
        self.clean["command"] = self.clean_coins
        self.clean.pack()
        
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] = self.quit
        self.QUIT.pack()

root = Tk()
title = "SmartMint"
size = "500x450"
conveyor1 = Conveyor(stepper1_pins)
conveyor2 = Conveyor(stepper2_pins)
feeder = Feeder(feeder_pin)
sorter = Sorter(sorter_pin)
app = GUI(root, title, size, conveyor1, conveyor2, feeder, sorter)

t1 = threading.Thread(target=conveyor1.run1)
t2 = threading.Thread(target=conveyor2.run2)
t1.setDaemon(True)
t2.setDaemon(True)
t1.start()
t2.start()

app.mainloop()
conveyor1.finish()
conveyor2.finish()
time.sleep(1)
GPIO.cleanup()
root.destroy()