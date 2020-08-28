from innermodelnode import InnerModelNode

class InnerModelMesh (InnerModelNode):
    def __init__ (self, id, meshPath, scalex, scaley, scalez,
                render, tx, ty, tz, rx, ry, rz, collidable, parent):
        super (InnerModelMesh, self).__init__ (id, parent)
        self.meshPath = meshPath
        self.scalex = scalex
        self.scaley = scaley
        self.scalez = scalez
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.renderMode = render # may be redundant
        self.collidable = collidable # may be redundant

    def __repr__ (self):
        scale = "scale: [{}, {}, {}]".format (self.scalex, self.scaley, self.scalez)
        translation = "translation: [{}, {}, {}]".format (self.tx, self.ty, self.tz)
        rotation = "rotation: [{}, {}, {}]".format (self.rx, self.ry, self.rz)
        meshPath = "meshPath: {}".format (self.meshPath)

        ret = "InnerModelMesh, id: {}, ".format (self.id) + scale + ", " + translation + ", " + \
                                                                        rotation + ", " + meshPath
        return ret

    def save (self, out, tabs):
        s = ""
        for _ in range (tabs):
            s += "\t"

        s += "<mesh id=\"" + self.id + "\"" + " file=\"" + self.meshPath + "\" scale=\"" + \
             "%.3f" % self.scalex + "," + "%.3f" % self.scalex + "," + "%.3f" % self.scalex + \
             "\" tx=\"" + "%.3f" % self.tx + "\" ty=\"" + "%.3f" % self.ty + "\" tz=\"" + \
             "%.3f" % self.tz + "\" rx=\"" + "%.3f" % self.rx + "\" ry=\"" + "%.3f" % self.ry + \
             "\" rz=\"" + "%.3f" % self.rz + "\" collide=\"" + "%.3f" % self.collidable + "\" />\n"
        out.write (s)

    def print (self, verbose): # may be redundant
        if (verbose):
            print (self)

    def setScale (self, x, y, z):
        self.scalex = x
        self.scaley = y
        self.scalez = z

    def normalRendering (self) -> bool: # may be redundant
        return self.renderMode == 'NormalRendering'

    def wireFrameRendering (self) -> bool: # may be redudant
        return self.renderMode == 'WireFrameRendering'
