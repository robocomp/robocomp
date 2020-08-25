from innermodeltransform import InnerModelTransform

class InnerModelPrismaticJoint (InnerModelTransform):
    def __init__(self, id: str, min: float, max: float, value: float, offset: float, port: int = 0,
                 axis: str = 'z', home: float = 0, parent: 'InnerModelTransform' = None):
        super (InnerModelPrismaticJoint, self).__init__ (id, 'static', 0, 0, 0, 0, 0, 0, 0, parent)
        self.value = value
        self.offset = offset
        self.min = min
        self.max = max
        self.home = home
        self.port = port
        self.axis = axis
        self.setPosition (value)

    def __repr__ (self):
        s1 = "InnerModelPrismaticJoint, id: {}, offset: {}, min: {}, max: {}, home: {}, port: {}"\
              .format (self.id, self.offset, self.min, self.max, self.home, self.port)
        s2 = ", axis: {}, pos: [{}, {}, {}], orient: [{}, {}, {}]".format (self.axis, self.tx, \
              self.ty, self.tz, self.rx, self.ry, self.rz)
        return s1 + s2

    def print (self, verbose): # redundant
        if verbose:
            print (self)

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<!-- joints cannot be saved yet -->\n"
        out.write (s)

    def getPosition (self) -> float:
        return self.value

    def setPosition (self, v: float) -> float:
        ret = None
        if v <= self.max and v >= min:
            ret = v
        else:
            if v > self.max:
                ret = self.max
            else:
                ret = self.min
        self.value = v = ret
        if self.axis is 'x':
            self.rtmat.set (0, 0, 0, v+self.offset, 0, 0)
        elif self.axis is 'y':
            self.rtmat.set (0, 0, 0, 0, v+self.offset, 0)
        elif self.axis is 'z':
            self.rtmat.set (0, 0, 0, 0, 0, v+self.offset)
        else:
            raise Exception ("internal error, no such axis %s" % self.axis)
        return ret
