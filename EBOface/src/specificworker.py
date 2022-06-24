#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *

from PIL import Image, ImageDraw
import threading
import sys, os, traceback, time, copy, json
from random import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

res_x = 1280
res_y = 800

fact_x = res_x / 480
fact_y = res_y / 320

# print(fact_x)
# print(fact_y)


configPath = os.path.join(os.path.dirname(os.path.dirname(__file__)),'etc','config')
# DEFAULTCONFIGNEUTRAL = {"cejaD": {"P2": {"y": 73, "x": 314}, "P3": {"y": 99, "x": 355}, "P1": {"y": 99, "x": 278}, "P4": {"y": 94, "x": 313}}, 
# "cejaI": {"P2": {"y": 73, "x": 160}, "P3": {"y": 99, "x": 201}, "P1": {"y": 99, "x": 122}, "P4": {"y": 94, "x": 160}}, 
# "ojoD": {"Radio1": {"Value": 34}, "Center": {"y": 151, "x": 316}, "Radio2": {"Value": 34}}, 
# "ojoI": {"Radio1": {"Value": 34}, "Center": {"y": 151, "x": 161}, "Radio2": {"Value": 34}}, 
# "boca": {"P2": {"y": 231, "x": 239}, "P3": {"y": 234, "x": 309}, "P1": {"y": 234, "x": 170}, "P6": {"y": 242, "x": 170}, "P4": {"y": 242, "x": 309}, "P5": {"y": 241, "x": 239}}, 
# "pupilaD": {"Radio": {"Value": 5}, "Center": {"y": 151, "x": 316}}, 
# "pupilaI": {"Radio": {"Value": 5}, "Center": {"y": 151, "x": 161}}, 
# "lengua": {"P2": {"y": 238, "x": 239}, "P3": {"y": 238, "x": 309}, "P1": {"y": 238, "x": 199}, "P4": {"y": 238, "x": 273}}, 
# "mejillaD": {"P2": {"y": 188, "x": 314}, "P3": {"y": 187, "x": 355}, "P1": {"y": 187, "x": 278}, "P4": {"y": 187, "x": 313}},
# "mejillaI": {"P2": {"y": 188, "x": 160}, "P3": {"y": 187, "x": 201}, "P1": {"y": 187, "x": 122}, "P4": {"y": 187, "x": 160}}, 
# "parpadoD": {"P2": {"y": 80, "x": 314}, "P3": {"y": 151, "x": 369}, "P1": {"y": 151, "x": 266}, "P4": {"y": 80, "x": 313}}, 
# "parpadoI": {"P2": {"y": 80, "x": 160}, "P3": {"y": 151, "x": 214}, "P1": {"y": 151, "x": 112}, "P4": {"y": 80, "x": 160}}} 

DEFAULTCONFIGNEUTRAL = {"cejaD": {"P2": {"y": 73*fact_y, "x": 314*fact_x}, "P3": {"y": 99*fact_y, "x": 355*fact_x}, "P1": {"y": 99*fact_y, "x": 278*fact_x}, "P4": {"y": 94*fact_y, "x": 313*fact_x}}, 
"cejaI": {"P2": {"y": 73*fact_y, "x": 160*fact_x}, "P3": {"y": 99*fact_y, "x": 201*fact_x}, "P1": {"y": 99*fact_y, "x": 122*fact_x}, "P4": {"y": 94*fact_y, "x": 160*fact_x}}, 
"ojoD": {"Radio1": {"Value": 34*fact_x}, "Center": {"y": 151*fact_y, "x": 316*fact_x}, "Radio2": {"Value": 34*fact_x}}, 
"ojoI": {"Radio1": {"Value": 34*fact_x}, "Center": {"y": 151*fact_y, "x": 161*fact_x}, "Radio2": {"Value": 34*fact_x}}, 
"boca": {"P2": {"y": 231*fact_y, "x": 239*fact_x}, "P3": {"y": 234*fact_y, "x": 309*fact_x}, "P1": {"y": 234*fact_y, "x": 170*fact_x}, "P6": {"y": 242*fact_y, "x": 170*fact_x}, "P4": {"y": 242*fact_y, "x": 309*fact_x}, "P5": {"y": 241*fact_y, "x": 239*fact_x}}, 
"pupilaD": {"Radio": {"Value": 5*fact_x}, "Center": {"y": 151*fact_y, "x": 316*fact_x}}, 
"pupilaI": {"Radio": {"Value": 5*fact_x}, "Center": {"y": 151*fact_y, "x": 161*fact_x}}, 
"lengua": {"P2": {"y": 238*fact_y, "x": 239*fact_x}, "P3": {"y": 238*fact_y, "x": 309*fact_x}, "P1": {"y": 238*fact_y, "x": 199*fact_x}, "P4": {"y": 238*fact_y, "x": 273*fact_x}}, 
"mejillaD": {"P2": {"y": 188*fact_y, "x": 314*fact_x}, "P3": {"y": 187*fact_y, "x": 355*fact_x}, "P1": {"y": 187*fact_y, "x": 278*fact_x}, "P4": {"y": 187*fact_y, "x": 313*fact_x}},
"mejillaI": {"P2": {"y": 188*fact_y, "x": 160*fact_x}, "P3": {"y": 187*fact_y, "x": 201*fact_x}, "P1": {"y": 187*fact_y, "x": 122*fact_x}, "P4": {"y": 187*fact_y, "x": 160*fact_x}}, 
"parpadoD": {"P2": {"y": 80*fact_y, "x": 314*fact_x}, "P3": {"y": 151*fact_y, "x": 369*fact_x}, "P1": {"y": 151*fact_y, "x": 266*fact_x}, "P4": {"y": 80*fact_y, "x": 313*fact_x}}, 
"parpadoI": {"P2": {"y": 80*fact_y, "x": 160*fact_x}, "P3": {"y": 151*fact_y, "x": 214*fact_x}, "P1": {"y": 151*fact_y, "x": 112*fact_x}, "P4": {"y": 80*fact_y, "x": 160*fact_x}}} 



OFFSET = 0.06666666666666667
def bezier(p1, p2, t):
	diff = (p2[0] - p1[0], p2[1] - p1[1])
	return [p1[0] + diff[0] * t, p1[1] + diff[1] * t]

def getPointsBezier(points):
	bezierPoints = list()
	pointsCopy = copy.copy(points)
	for t in [x/50. for x in range(51)]:
		while len(points)!=1:
			newPoints = list()
			p1=points[0]
			for p2 in points[1:]:
				newPoints.append(bezier(p1,p2,t))
				p1=p2
			points=newPoints
		bezierPoints.append(tuple(points[0]))
		points=pointsCopy
	return bezierPoints

def getBecierConfig(old_config, config_target, t):
	config = copy.copy(old_config)
	for parte in old_config:
		for point in old_config[parte]:
			if "Radio" in point:
				radio = bezier((old_config[parte][point]["Value"],0), (config_target[parte][point]["Value"],0),t)
				config[parte][point]["Value"] = radio[0]
			else:
				p = bezier((old_config[parte][point]["x"], old_config[parte][point]["y"]), (config_target[parte][point]["x"], config_target[parte][point]["y"]), t)
				config[parte][point]["x"] = p[0]
				config[parte][point]["y"] = p[1]
	return config


class Face(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.img = Image.new('RGBA', (res_x, res_y), (0, 0, 0))
        self.background = Image.new('RGBA', (res_x, res_y), (255, 255, 255))

        self.pup_x = 0
        self.pup_y = 0


        self.draw = ImageDraw.Draw(self.img)
        self.config = DEFAULTCONFIGNEUTRAL
        self.old_config = DEFAULTCONFIGNEUTRAL
        self.t = 0.9
        self.config_target = DEFAULTCONFIGNEUTRAL
        self.stopped = False
        self.isTalking = False
        self.isListening = False
        self.pupilaFlag = False
        self.val_lim = 20*fact_x

        self.val_lim_x = 20*fact_x
        self.val_lim_y = 20*fact_x
        # self.pupilaFlag = False

    def run(self):
        start = time.time()
        sec = randint(2,6)
        while not self.stopped:
            # time.sleep(0.1)
            pestaneoFlag = False
            if time.time() - start > sec:

                pestaneoFlag = True
                sec = randint(2, 6)
                start = time.time()
                # print("entro")

            self.moveFace(pestaneoFlag, self.isTalking, self.isListening)

            path = self.render()
            if self.isListening:
                self.recordPoint()
            if self.pupilaFlag:
                self.movePupila()
                print("asdfartgadfgadfg", self.pup_x)
                print("asdfartgadfgadfg", self.pup_y)                
            if path is not None:
                path = "/dev/fb0"
                rotated_image = self.img.rotate(270)
                area = (280, 0 ,1000, res_y-1)
                cropped_img = rotated_image.crop(area)
                self.background.paste(cropped_img,(300,0))
                with open(path, "wb") as f:
                    f.write(self.background.tobytes())
                #self.display_proxy.setImageFromFile(path)

    def movePupila(self):
        configaux = copy.copy(self.config)

        # PUPILA

        valuel1 = 161*fact_x
        valuel2 = 151*fact_y
        valuer1 = 316*fact_x
        valuer2 = 151*fact_y

        print("val",valuel1)


        configaux["pupilaI"]["Center"]["x"] = valuel1+self.val_lim*self.pup_x
        configaux["pupilaI"]["Center"]["y"] = valuel2+self.val_lim*self.pup_y
        configaux["pupilaD"]["Center"]["x"] = valuer1+self.val_lim*self.pup_x
        configaux["pupilaD"]["Center"]["y"] = valuer2+self.val_lim*self.pup_y

        print(configaux["pupilaI"]["Center"]["x"])
        print(configaux["pupilaI"]["Center"]["y"])
        print(configaux["pupilaD"]["Center"]["x"])
        print(configaux["pupilaD"]["Center"]["y"])
        
        # config1 = getBecierConfig(configaux, configPestaneo, t)
        self.drawConfig(configaux)
        # self.img = self.img.rotate(270)
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())
        # img = np.array(self.img)
        # img = cv2.flip(img, 1)
        # cv2.imwrite("/tmp/ebofaceimg.png", img)
        # self.display_proxy.setImageFromFile("/tmp/ebofaceimg.png")



    def moveFace(self, pestaneoFlag, moveMouthFlag, listeningFlag):
        if not pestaneoFlag and not moveMouthFlag and not listeningFlag:
            return

        configaux = copy.copy(self.config)

        # PESTAÑEO
        value1 = copy.copy((configaux["ojoD"]["Radio2"]["Value"]))
        value2 = copy.copy((configaux["ojoI"]["Radio2"]["Value"]))

        # PUPILA

        valuel1 = copy.copy((configaux["pupilaI"]["Center"]["x"]))
        valuel2 = copy.copy((configaux["pupilaI"]["Center"]["y"]))
        valuer1 = copy.copy((configaux["pupilaD"]["Center"]["x"]))
        valuer2 = copy.copy((configaux["pupilaD"]["Center"]["y"]))

        # BOCA
        value2x = copy.copy((configaux["boca"]["P2"]["x"]))
        value2y = copy.copy((configaux["boca"]["P2"]["y"]))

        value5x = copy.copy((configaux["boca"]["P5"]["x"]))
        value5y = copy.copy((configaux["boca"]["P5"]["y"]))

        limiteojosY = uniform(-self.val_lim, self.val_lim)
        limiteojosX = uniform(-self.val_lim, self.val_lim)

        compr = bezier((valuel1, valuel2), (valuel1 + limiteojosX, valuel2 + limiteojosY), 1)
        if ((-20*fact_x < abs(compr[0]) - 161*fact_x < 20*fact_x) and (-20*fact_x < abs(compr[1]) - 151*fact_x < 20*fact_x)):
            pupilaFlag = True
        else:
            pupilaFlag = False
        aux_t = 0

        for t in [(x+1)/5. for x in range(5)] + sorted([(x)/5. for x in range(5)], reverse=True):
            # print(t)

            if pestaneoFlag:

                    # print(abs(compr[0])- 161)
                    # print(abs(compr[1])- 151)

                if self.pupilaFlag == False and pupilaFlag == True:

                    pupI = bezier((valuel1, valuel2), (valuel1 + limiteojosX, valuel2 + limiteojosY), aux_t)
                    pupD = bezier((valuer1, valuer2), (valuer1 + limiteojosX, valuer2 + limiteojosY), aux_t)

                    configaux["pupilaI"]["Center"]["x"] = pupI[0]
                    configaux["pupilaI"]["Center"]["y"] = pupI[1]
                    configaux["pupilaD"]["Center"]["x"] = pupD[0]
                    configaux["pupilaD"]["Center"]["y"] = pupD[1]

                    aux_t += 0.1

                configaux["ojoD"]["Radio2"]["Value"] = bezier((value1,0), (0,0), t)[0]
                configaux["ojoI"]["Radio2"]["Value"] = bezier((value2, 0), (0, 0), t)[0]


            if moveMouthFlag:
                configaux["boca"]["P2"]["y"] = bezier((value2x, value2y), (value2x, value2y - 30*fact_y), t)[1]
                configaux["boca"]["P5"]["y"] = bezier((value5x, value5y), (value5x, value5y + 30*fact_y), t)[1]
            # config1 = getBecierConfig(configaux, configPestaneo, t)
            self.drawConfig(configaux)
            # self.img = self.img.rotate(270)
            path = "/dev/fb0"
            rotated_image = self.img.rotate(270)
            area = (280, 0 ,1000, res_y-1)
            cropped_img = rotated_image.crop(area)
            self.background.paste(cropped_img,(300,0))
            with open(path, "wb") as f:
                f.write(self.background.tobytes())
            # img = np.array(self.img)
            # img = cv2.flip(img, 1)
            # cv2.imwrite("/tmp/ebofaceimg.png", img)
            # self.display_proxy.setImageFromFile("/tmp/ebofaceimg.png")

    def drawConfig(self, config):
        self.draw.rounded_rectangle(((0, 0), (res_x-1, res_y-1)),fill=(255, 255, 255), outline=(0, 0, 0))
        self.renderOjo(config["ojoI"])
        self.renderOjo(config["ojoD"])
        self.renderParpado(config["parpadoI"])
        self.renderParpado(config["parpadoD"])
        self.renderCeja(config["cejaI"])
        self.renderCeja(config["cejaD"])
        self.renderBoca(config["boca"])
        self.renderPupila(config["pupilaI"])
        self.renderPupila(config["pupilaD"])
        self.renderMejilla(config["mejillaI"])
        self.renderMejilla(config["mejillaD"])
        self.renderLengua(config["lengua"])
        # if self.isListening:
        #     P1 = (640 - 10, 200 - 10)
        #     P2 = (640 + 10, 200 + 10)
        #     self.draw.ellipse((P1, P2), fill=1)

    def render(self):
        if self.t <= 1 and self.config_target is not None:
            config = self.config = getBecierConfig(self.old_config, self.config_target, self.t)

            self.drawConfig(config)
            self.t += OFFSET
            path = "/dev/fb0"
            rotated_image = self.img.rotate(270)
            area = (280, 0 ,1000, res_y-1)
            cropped_img = rotated_image.crop(area)
            self.background.paste(cropped_img,(300,0))
            with open(path, "wb") as f:
                f.write(self.background.tobytes())
            # return path
        else:
       # elif self.config_target is not None:
            # with self.mutex:
             self.old_config = self.config_target
             self.config_target = None
        return None

    def renderPupila(self, points):
        P1 = (points["Center"]["x"] - points["Radio"]["Value"], points["Center"]["y"] - points["Radio"]["Value"])
        P2 = (points["Center"]["x"] + points["Radio"]["Value"], points["Center"]["y"] + points["Radio"]["Value"])
        self.draw.ellipse((P1, P2), fill=(255, 255, 255), outline=(255, 255, 255))

    # self.draw.ellipse((P1, P2), fill=1)

    def renderLengua(self, points):
        P1 = (points["P1"]["x"], points["P1"]["y"])
        P2 = (points["P2"]["x"], points["P2"]["y"])
        P3 = (points["P3"]["x"], points["P3"]["y"])
        P4 = (points["P4"]["x"], points["P4"]["y"])
        self.draw.polygon(getPointsBezier([P1, P2, P3, P4]), fill=(131,131,255), outline=(0,0,0))

    def renderParpado(self, points):
        P1 = (points["P1"]["x"], points["P1"]["y"])
        P2 = (points["P2"]["x"], points["P2"]["y"])
        P3 = (points["P3"]["x"], points["P3"]["y"])
        P4 = (points["P4"]["x"], points["P4"]["y"])
        self.draw.polygon(getPointsBezier([P1, P2, P3]) + getPointsBezier([P3, P4, P1]), fill=(255, 255, 255))

    def renderMejilla(self, points):
        P1 = (points["P1"]["x"], points["P1"]["y"])
        P2 = (points["P2"]["x"], points["P2"]["y"])
        P3 = (points["P3"]["x"], points["P3"]["y"])
        P4 = (points["P4"]["x"], points["P4"]["y"])
        self.draw.polygon(getPointsBezier([P1, P2, P3]) + getPointsBezier([P3, P4, P1]), fill=(255, 255, 255))

    def renderCeja(self, points):
        P1 = (points["P1"]["x"], points["P1"]["y"])
        P2 = (points["P2"]["x"], points["P2"]["y"])
        P3 = (points["P3"]["x"], points["P3"]["y"])
        P4 = (points["P4"]["x"], points["P4"]["y"])
        self.draw.polygon(getPointsBezier([P1, P2, P3]) + getPointsBezier([P3, P4, P1]), fill=1)

    def renderOjo(self, points):
        P1 = (points["Center"]["x"] - points["Radio1"]["Value"], points["Center"]["y"] - points["Radio2"]["Value"])
        P2 = (points["Center"]["x"] + points["Radio1"]["Value"], points["Center"]["y"] + points["Radio2"]["Value"])
        # P1 = (points["P1"]["x"], points["P1"]["y"])
        # P2 = (points["P2"]["x"], points["P2"]["y"])
        self.draw.ellipse((P1, P2), fill=1)

    def renderBoca(self, points):
        P1 = (points["P1"]["x"], points["P1"]["y"])
        P2 = (points["P2"]["x"], points["P2"]["y"])
        P3 = (points["P3"]["x"], points["P3"]["y"])
        P4 = (points["P4"]["x"], points["P4"]["y"])
        P5 = (points["P5"]["x"], points["P5"]["y"])
        P6 = (points["P6"]["x"], points["P6"]["y"])
        self.draw.polygon(getPointsBezier([P1, P2, P3]) + getPointsBezier([P4, P5, P6]), fill=1, outline=10)

    def recordPoint(self):

        P1 = (res_x*2/5 - 20, res_y/7 - 20)
        P2 = (res_x*2/5 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=1, outline=1)
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

        P1 = (res_x/2 - 20, res_y/7 - 20)
        P2 = (res_x/2 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=1, outline=1)
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

        P1 = (res_x*3/5 - 20, res_y/7 - 20)
        P2 = (res_x*3/5 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=1, outline=1)
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

        P1 = (res_x*2/5 - 20, res_y/7 - 20)
        P2 = (res_x*2/5 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=(255,255,255), outline=(255,255,255))
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

        P1 = (res_x/2 - 20, res_y/7 - 20)
        P2 = (res_x/2 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=(255,255,255), outline=(255,255,255))
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

        P1 = (res_x*3/5 - 20, res_y/7 - 20)
        P2 = (res_x*3/5 + 20, res_y/7 + 20)
        self.draw.ellipse((P1, P2), fill=(255,255,255), outline=(255,255,255))
        path = "/dev/fb0"
        rotated_image = self.img.rotate(270)
        area = (280, 0 ,1000, res_y-1)
        cropped_img = rotated_image.crop(area)
        self.background.paste(cropped_img,(300,0))
        with open(path, "wb") as f:
            f.write(self.background.tobytes())

        time.sleep(0.2)

    def setConfig(self, config):
        # with self.mutex:
        self.config_target = config
        self.old_config = self.config
        self.t = 0.06666666666666667
        # self.start()

    def setTalking(self, t):
        self.isTalking = t

    def setListening(self, t):
        self.isListening = t

    def stop(self):
        self.stopped = True
        self.join()


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 10
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        self.face = Face()
        self.start_time = time.time()
        self.sec = randint(2,6)
        self.configEmotions = {}
        json_path = os.path.join(os.path.dirname(__file__),'../JSON')
        for path in os.listdir(json_path):
            with open(os.path.join(json_path, path), "r") as f:
                self.configEmotions[os.path.splitext(path)[0]] = json.loads(f.read())
        for config in self.configEmotions:
            for part in self.configEmotions[config]:
                for point in self.configEmotions[config][part]:
                    if "Radio" in point:
                        self.configEmotions[config][part][point]["Value"] = self.configEmotions[config][part][point]["Value"]*fact_x
                    else:
                        self.configEmotions[config][part][point]["x"] = self.configEmotions[config][part][point]["x"] * fact_x
                        self.configEmotions[config][part][point]["y"] = self.configEmotions[config][part][point]["y"] * fact_y
        self.face.start()


    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        pass


    @QtCore.Slot()
    def compute(self):
        # print(self.face.pup_x)
        pass

	#
	# expressFear
	#
    def expressFear(self):
        self.face.setConfig(self.configEmotions["Fear"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/miedo.fb")

	#
	# expressSurprise
	#
    def expressSurprise(self):
        self.face.setConfig(self.configEmotions["Surprise"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/sorpresa.fb")

	#
	# expressAnger
	#
    def expressAnger(self):
        self.face.setConfig(self.configEmotions["Anger"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/ira.fb")

    #
    # expressSadness
    #
    def expressSadness(self):
        self.face.setConfig(self.configEmotions["Sadness"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/tristeza.fb")

    #
    # expressDisgust
    #
    def expressDisgust(self):
        self.face.setConfig(self.configEmotions["Disgust"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/asco.fb")

    #
    # expressJoy
    #
    def expressJoy(self):
        self.face.setConfig(self.configEmotions["Joy"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/alegria.fb")

    #
    # expressNeutral
    #
    def expressNeutral(self):
        self.face.setConfig(self.configEmotions["Neutral"])
        # self.display_proxy.setImageFromFile("/home/robocomp/learnbot/learnbot_components/emotionalMotor/imgs/frameBuffer/SinEmocion2.fb")

    def talking(self, t):
        self.face.setTalking(t)

    def listening(self, t):
        self.face.setListening(t)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    #
    # IMPLEMENTATION of isanybodythere method from EmotionalMotor interface
    #
    def EmotionalMotor_isanybodythere(self, isAny):
        if isAny == True:
            self.face.pupilaFlag = True
            print("activo")
        else:
            self.face.pupilaFlag = False
            print("inactivo")



    #
    # IMPLEMENTATION of pupposition method from EmotionalMotor interface
    #
    def EmotionalMotor_pupposition(self, x, y):
        self.face.pup_x = x
        self.face.pup_y = y




    # ===================================================================
    # ===================================================================



