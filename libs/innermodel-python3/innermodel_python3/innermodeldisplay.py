from innermodelnode import InnerModelNode
from innermodelvector import InnerModelVector

class InnerModelDisplay (InnerModelNode):
    def __init__ (self, id: str, port: int, texture: str, width: float, height: float, depth: float,
                  repeat: int, nx: float, ny: float, nz: float, px: float, py: float, pz: float,
                  collidable: bool, parent: 'InnerModelNode' = None):
        super (InnerModelDisplay, self).__init__ (id, parent)
        self.normal = InnerModelVector.vec3d (nx, ny, nz)
        self.point = InnerModelVector.vec3d (px, py, pz)
        self.port = port
        self.repeat = repeat # redundant
        self.texture = texture
        self.width = width
        self.height = height
        self.depth = depth
        self.collidable = collidable # redundant

    def __repr__ (self):
        normal = "normal: [{}, {}, {}]".format (self.normal[0], self.normal[1], self.normal[2])
        point = "point: [{}, {}, {}]".format (self.point[0], self.point[1], self.point[2])
        texture = "texture: {}".format (self.texture)
        width = "width: {}".format (self.width)
        height = "height: {}".format (self.height)
        depth = "depth: {}".format (self.depth)

        ret = "InnerModelDisplay, id: {}, ".format (self.id) + normal + ", " + point + ", " + \
               texture + ", " + width + ", " + height + ", " + depth
        return ret

    def updateTexture (self, texture: str):
        self.texture = texture

    def print (self, verbose: bool): # redundant
        if verbose:
            print (self)

    def save (self, out, tabs: int):
        s = ""
        for _ in range (tabs):
            s += "\t"
        s += "<display id=\"" + self.id + "\" texture=\"" + self.texture + "\" size=\"" + \
             "%.3f" % self.width + "," + "%.3f" % self.height + "," + "%.3f" % self.depth + \
             "\" repeat=\"" + "%.3f" % self.repeat + "\" nx=\"" + "%.3f" % self.normal[0] + \
             "\" ny=\"" + "%.3f" % self.normal[1] + "\" nz=\"" + "%.3f" % self.normal[2] + \
             "\" px=\"" + "%.3f" % self.point[0] + "\" py=\"" + "%.3f" % self.point[1] + \
             "\" pz=\"" + "%.3f" % self.point[2] + "\" collide=\"" + "%.3f" % self.collidable + \
             "\"" + " port=\"" + self.port + "\" />\n"
        out.write (s)

    def update (self, nx: float = None, ny: float = None, nz: float = None, px: float = None,
                py: float = None, pz: float = None):
        self.normal[0] = nx; self.normal[1] = ny; self.normal[2] = nz
        self.point[0] = nx; self.point[1] = ny; self.point[2] = nz
