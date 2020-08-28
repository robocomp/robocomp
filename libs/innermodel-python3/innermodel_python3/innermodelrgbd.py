from innermodelcamera import InnerModelCamera

class InnerModel (object):
    pass

class InnerModelRGBD (InnerModelCamera):
    def __init__(self, id: str, width: float, height: float, focal: float, noise: float, port: int,
                 ifconfig: str, innermodel: 'InnerModel', parent: 'InnerModelNode' = None):
        super (InnerModelRGBD, self).__init__ (id, width, height, focal, innermodel, parent)
        self.noise = noise
        self.port = port
        self.ifconfig = ifconfig # redundant

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<rgbd id=\"" + self.id + "\" width=\"" + "%.3f" % self.width + "\" height=\"" + \
             "%.3f" % self.height + "\" focal=\"" + "%.3f" % self.focal + "\" port=\"" + self.port \
             + "\" ifconfig=\"" + self.ifconfig + "\" noise=\"" + "%.3f" % self.noise + "\" />\n"
        out.write (s)
