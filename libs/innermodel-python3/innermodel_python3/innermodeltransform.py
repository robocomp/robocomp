'''
Class representing transformation (tx, ty, tz, rx, ry, rz)
Author: ksakash@github.com (Akash Kumar Singh)
'''

from innermodelnode import InnerModelNode
from innermodelrtmatrix import InnerModelRTMat

class InnerModelTransform(InnerModelNode):
    '''Class representing transformation (tx, ty, tz, rx, ry, rz)'''

    def __init__ (self, id: str, engine: str, tx: float, ty: float, tz: float, rx: float, ry: float,
                  rz: float, mass: float = 0, parent: 'InnerModelNode' = None):

        super (InnerModelTransform, self).__init__(id, parent)
        self.rtmat = InnerModelRTMat.getInnerModelRTMat (tx=tx, ty=ty, tz=tz, rx=rx, ry=ry, rz=rz)
        self.mass = mass

        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz

        self.gui_translation = True # may be redundant
        self.gui_rotation = True # may be redundant

        self.engine = engine # may be redundant

    def __repr__ (self):
        ret1 = "InnerModelTransform, id: {}, mass: {}".format (self.id, self.mass)
        ret2 = ", tx: {}, ty: {}, tz: {}, rx: {}, ry: {}, rz: {}".format (self.tx, self.ty, \
                                                                self.tz, self.rx, self.ry, self.rz)
        return ret1 + ret2

    # print the rt matrix
    def print (self, verbose): # redundant if __repr__ is used
        '''Print info about object'''

        if verbose:
            print (self)

    def save (self, out, tabs):
        '''Save info to a doc'''

        s = ""

        if self.id == "root":
            for _ in range (tabs):
                s += "\t"

            s += "<innermodel>\n"
            out.write (s)

            for child in self.children:
                child.save(out, tabs+1)

            s = ""
            for _ in range (tabs):
                s += "\t"

            s += "</innermodel>\n"
            out.write (s)
        else:
            for _ in range (tabs):
                s += "\t"

            s += "<transform id=\"" + self.id + "\" tx=\"" + "%.3f" % self.tx + "\" ty=\"" + \
                 "%.3f" % self.ty + "\" tz=\"" + "%.3f" % self.tz + "\"  rx=\"" + "%.3f" % self.rx \
                 + "\" ry=\"" + "%.3f" % self.ry + "\" rz=\"" + "%.3f" % self.rz + "\">\n"
            out.write (s)

            for child in self.children:
                child.save (out, tabs+1)

            s = ""
            for _ in range (tabs):
                s += "\t"

            s += "</transform>\n"
            out.write (s)

    def updateTranslation (self, tx, ty, tz): # may be redundant
        '''Update translation parameters'''

        self.tx = tx
        self.ty = ty
        self.tz = tz

    def updateRotation (self, rx, ry, rz): # may be redundant
        '''Update rotation parameters'''

        self.rx = rx
        self.ry = ry
        self.rz = rz

    def update (self, tx=None, ty=None, tz=None, rx=None, ry=None, rz=None):
        '''Update the paramters of the object'''

        if tx is not None:
            self.tx = tx
        if ty is not None:
            self.ty = ty
        if tz is not None:
            self.tz = tz
        if rx is not None:
            self.rx = rx
        if ry is not None:
            self.ry = ry
        if rz is not None:
            self.rz = rz

        self.rtmat.set (ox=self.rx, oy=self.ry, oz=self.rz, x=self.tx, y=self.ty, z=self.tz)
