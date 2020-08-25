import sys
import time
from math import cos, sin

sys.path.append('../innermodel_python3')

from innermodel import InnerModel
from innermodelviewer import InnerModelViewer

im = InnerModel (xmlFilePath='test2.xml')
imv = InnerModelViewer (innermodel=im)

robot_transform = im.getTransform('robot')
alpha = 0
while True:

    print(f"alpha:{alpha}")
    robot_transform.update(tx=100.*sin(alpha)+100., ty=100.*cos(alpha), rz=alpha)
    alpha += 0.001
    print(f"Position of world from robot's perspective: {im.transform('robot', 'world')}")
    print(f"Position of robot from world's perspective: {im.transform('world', 'robot')}")
    imv.render()
    time.sleep(1. / 30.)
