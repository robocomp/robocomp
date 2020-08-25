from innermodelnode import InnerModelNode

# redundant
class InnerModelTouchSensor (InnerModelNode):
    def __init__ (self, id: str, stype: str, nx: float, ny: float, nz: float, min=-float("inf"),
                  max=float("inf"), port: int = 0, parent: 'InnerModelNode' =None):
        super (InnerModelTouchSensor, self).__init__ (id, parent)
        self.nx = nx; self.ny = ny; self.nz = nz
        self.min = min; self.max = max
        self.stype = stype
        self.port = port
        self.value = None

    def __repr__ (self):
        s1 = "InnerModelTouchSensor, id: {}, normal: [{}, {}, {}], min: {}, max: {}, stype: {}"\
             .format (self.id, self.nx, self.ny, self.nz, self.min, self.max, self.stype)
        s2 = ", port: {}".format (self.port)
        return s1 + s2

    def print (self, verbose: bool): # redundant
        pass

    def save (self, out, tabs: int): # redundant
        pass

    def getMeasure (self) -> float:
        return self.value
