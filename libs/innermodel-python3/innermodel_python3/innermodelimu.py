from innermodelnode import InnerModelNode

class InnerModelIMU (InnerModelNode):
    def __init__ (self, id: str, port: int, parent: 'InnerModelNode' = None):
        super (InnerModelIMU, self).__init__ (id=id, parent=parent)
        self.port = port

    def __repr__ (self) -> str:
        ret = "InnerModelIMU, id: {}, port: {}".format (self.id, self.port)
        return ret

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<imu id=\"" + self.id + "\" />\n"
        out.write (s)

    def print (self, verbose: bool): # redundant
        if verbose:
            print (self)
