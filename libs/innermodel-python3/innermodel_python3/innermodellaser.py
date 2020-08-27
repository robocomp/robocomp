import math
from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector

class InnerModel (object):
    pass

class InnerModelLaser (InnerModelNode):
    def __init__ (self, id: str, port: int, min: int, max: int, angle: float, measures: int,
                  ifconfig: str, innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelLaser, self).__init__ (id, parent)
        self.port = port
        self.min = min
        self.max = max
        self.angle = angle
        self.measures = measures
        self.ifconfig = ifconfig # redundant
        self.innerModel = innermodel

    def __repr__ (self) -> str:
        ret = "InnerModelLaser, id: {}, port: {}, min: {}, max: {}, angle: {}, measures: {}".format \
                                (self.id, self.port, self.min, self.max, self.angle, self.measures)
        return ret

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<laser id=\"" + self.id + "\" port=\"" + self.port + "\" min=\"" + "%.3f" % self.min \
             + "\" max=\"" + "%.3f" % self.max + "\" measures=\"" + "%.3f" % self.measures + \
             "\" angle=\"" + "%.3f" % self.angle + "\" ifconfig=\"" + self.ifconfig + "\" />\n"
        out.write (s)

    def print (self, verbose: bool): # redundant
        if verbose:
            print (self)

    def laserTo (self, dest: str, r: float, alpha: float) -> 'InnerModelVector':
        p = InnerModelVector((3,))
        p[0] = r*math.sin (alpha)
        p[1] = 0
        p[2] = r*math.cos (alpha)
        return self.innerModel.transform (dest, self.id, p)
