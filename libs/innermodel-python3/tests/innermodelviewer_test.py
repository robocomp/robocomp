import sys
import time
from math import cos, sin

sys.path.append('../innermodel_python3')

from innermodel import InnerModel
from innermodelviewer import InnerModelViewer

im = InnerModel (xmlFilePath='test.xml')
imv = InnerModelViewer (innermodel=im)

base_transform = im.getTransform('base')
alpha = 0
while True:
    base_transform.update(tx=sin(alpha), ty=cos(alpha), rz=alpha)
    alpha += 0.02
    print(im.transform('base', 'world'))
    imv.render()
    time.sleep(1. / 30.)
