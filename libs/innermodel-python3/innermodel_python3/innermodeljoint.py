'''
Class representing InnerModelJoint
Author: ksakash@github.com (Akash Kumar Singh)
'''

from innermodeltransform import InnerModelTransform
from innermodelvector import InnerModelVector

class InnerModelJoint (InnerModelTransform):
    def __init__ (self, id: str, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float,
                  tx: float, ty: float, tz: float, rx: float, ry: float, rz: float, min: float =-float("inf"),
                  max: float = float("inf"), port: int = 0, axis: str = "z", home: float = 0,
                  parent: 'InnerModelTransform' = None):

        super (InnerModelJoint, self).__init__ (id, 'static', tx, ty, tz, rx, ry, rz, 0, parent)
        self.min = min
        self.max = max
        self.home = home
        self.port = port
        self.axis = axis
        self.backl = None
        self.backh = None

        if axis is 'x':
            self.backl = lx
            self.backh = hx
            self.update (min, 0, 0, max, 0, 0)
        elif axis is 'y':
            self.backl = ly
            self.backh = hy
            self.update (0, min, 0, 0, max, 0)
        elif axis is 'z':
            self.backl = lz
            self.backh = hz
            self.update (0, 0, min, 0, 0, max)
        else:
            raise Exception ("internal error, no such axis %s" % axis)

    def __repr__ (self):
        s1 = "InnerModelJoint, id: {}, pos: [{}, {}, {}], orient: [{}, {}, {}], lower_limit: {}," \
            .format (self.id , self.tx, self.ty, self.tz, self.rx, self.ry, self.rz, self.backl)
        s2 = " higher_limit: {}, min: {}, max: {}, port: {}, axis: {}".format (self.backh, self.min, \
             self.max, self.port, self.axis)

        return s1 + s2

    def print (self, verbose: bool): # redundant
        '''Print info about the given node'''

        if verbose:
            print (self)

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<joint id=\"" + self.id + "\" port=\"" + self.port + "\" axis=\"" + self.axis + \
             "\" home=\"" + "%.3f" % self.home + "\" min=\"" + "%.3f" % self.min + "\" max=\"" + \
             "%.3f" % self.max + "\" tx=\"" + "%.3f" % self.tx + "\" ty=\"" + "%.3f" % self.tx + \
             "\" tz=\"" + "%.3f" % self.tz + "\" rx=\"" + "%.3f" % self.rx + "\" ry=\"" + \
             "%.3f" % self.ry + "\" rz=\"" + "%.3f" % self.rz + "\">\n"

        out.write (s)

        for child in self.children:
            child.save (out, tabs+1)

        s = ""

        for _ in range (tabs):
            s += "\t"

        s += "</joint>\n"
        out.write (s)

    def update (self, lx: float, ly: float, lz: float, hx: float, hy: float, hz: float):
        '''Update parameters about the given parameters'''

        if self.axis is 'x':
            self.backl = lx
            self.backh = hx
        elif self.axis is 'y':
            self.backl = ly
            self.backh = hy
        elif self.axis is 'z':
            self.backl = lz
            self.backh = hz

    def getAngle (self) -> float:
        '''Returns the angle of rotation about the given axis'''

        if (self.axis is 'x'):
            return self.rx
        elif (self.axis is 'y'):
            return self.ry
        elif (self.axis is 'z'):
            return self.rz

    def setAngle (self, angle: float, force: bool = False) -> float:
        '''Set angle of rotation'''

        ret = angle

        if (angle > self.max):
            ret = self.max
        elif (angle < self.min):
            ret = self.min

        if (self.axis == "x"):
            self.rx = ret
            self.rtmat.set (ret, 0, 0, 0, 0, 0)
        elif (self.axis == "y"):
            self.ry = ret
            self.rtmat.set(0, ret, 0, 0, 0, 0)
        elif (self.axis == "z"):
            self.rz = ret
            self.rtmat.set(0, 0, ret, 0, 0, 0)
        else:
            raise Exception ("internal error, no such axis %s", self.axis)

        if (self.innerModel is not None):
            self.innerModel.cleanupTables()

        return ret

    def uninaryAxis (self) -> 'InnerModelVector':
        if self.axis is 'x':
            return InnerModelVector.vec3d (1, 0, 0)
        elif self.axis is 'y':
            return InnerModelVector.vec3d (0, 1, 0)
        elif self.axis is 'z':
            return InnerModelVector.vec3d (0, 0, 1)
        return InnerModelVector.vec3d (0, 0, 0)
