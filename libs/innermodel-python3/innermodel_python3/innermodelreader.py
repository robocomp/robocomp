'''
Class to read a innermodel doc
Author: ksakash@github.com (Akash Kumar Singh)
'''

import xml.etree.ElementTree as ET
from innermodelnode import InnerModelNode
from innermodeltransform import InnerModelTransform
from innermodelcamera import InnerModelCamera

class InnerModel (object):
    pass

class InnerModelReader (object):
    '''Class to read an innermodel doc'''

    validAttr = dict()
    def __init__ (self):
        self.initNodeAttributes()

    @staticmethod
    def initNodeAttributes ():
        '''Method to store valid attributes for a tag'''

        attrList = []
        InnerModelReader.validAttr["innermodel"] = attrList

        attrList = ["id", "rx", "ry", "rz", "engine", "mass"]
        InnerModelReader.validAttr["rotation"] = attrList

        attrList = ["id", "tx", "ty", "tz", "engine", "mass"]
        InnerModelReader.validAttr["translation"] = attrList

        attrList = ["id", "tx", "ty", "tz", "rx", "ry", "rz", "engine", "mass"]
        InnerModelReader.validAttr["transform"] = attrList

        attrList = ["id", "lx", "ly", "lz", "hx", "hy", "hz", "tx", "ty", "tz", "rx", "ry", "rz",
                    "min", "max", "port", "axis", "home"]
        InnerModelReader.validAttr["joint"] = attrList

        attrList = ["id", "type", "nx", "ny", "nz", "min", "max", "port"]
        InnerModelReader.validAttr["touchsensor"] = attrList

        attrList = ["id", "min", "max", "position", "offset", "port", "axis", "home"]
        InnerModelReader.validAttr["prismaticjoint"] = attrList

        attrList = ["id", "tx", "ty", "tz", "rx", "ry", "rz", "port", "noise", "collide"]
        InnerModelReader.validAttr["differentialrobot"] = attrList

        attrList = ["id", "tx", "ty", "tz", "rx", "ry", "rz", "port", "noise", "collide"]
        InnerModelReader.validAttr["omnirobot"] = attrList

        attrList = ["id", "width", "height", "focal", "noise", "port", "ifconfig"]
        InnerModelReader.validAttr["rgbd"] = attrList

        attrList = ["id", "port"]
        InnerModelReader.validAttr["imu"] = attrList

        attrList = ["id", "port", "min", "max", "angle", "measures", "ifconfig"]
        InnerModelReader.validAttr["laser"] = attrList

        attrList = ["id", "width", "height", "focal"]
        InnerModelReader.validAttr["camera"] = attrList

        attrList =["id", "file", "scale", "render", "tx", "ty", "tz", "rx", "ry", "rz", "collide"]
        InnerModelReader.validAttr["mesh"] = attrList

        attrList = ["id"]
        InnerModelReader.validAttr["pointcloud"] = attrList

        attrList = ["id", "texture", "repeat", "size", "nx", "ny", "nz", "px", "py", "pz", "collide"]
        InnerModelReader.validAttr["plane"] = attrList

        attrList = ["id", "texture", "repeat", "size", "nx", "ny", "nz",
                    "px", "py", "pz", "collide", "port"]
        InnerModelReader.validAttr["display"] = attrList

        attrList = ["path"]
        InnerModelReader.validAttr["include"] = attrList

        attrList = ["id", "length", "width", "lengthx", "widthx", "lengthy", "widthy", "lengthz",
                    "widthz"]
        InnerModelReader.validAttr["axes"] = attrList

    @staticmethod
    def load (file: str, model: 'InnerModel') -> bool:
        '''Method to load an innermoel doc'''

        InnerModelReader.initNodeAttributes()
        tree = ET.parse (file)
        root = tree.getroot()
        if root.tag.lower() != 'innermodel':
            print ("<innerModel> tag is missing!!")
            return False
        if (model.root is None):
            r = InnerModelTransform ('root', 'static', 0, 0, 0, 0, 0, 0)
            model.setRoot (r)
            r.parent = None
        InnerModelReader.recursive (root, model, model.root)
        return True

    @staticmethod
    def include (file: str, model: 'InnerModel', node: 'InnerModelNode') -> bool:
        '''Method to load innermodel doc included in other innermodel doc'''

        tree = ET.parse (file)
        root = tree.getroot ()
        if root.tag.lower() != 'innermodel':
            print ("<innerModel> tag is missing!!")
            return False
        if (model.root is None):
            r = InnerModelTransform ('root', 'static', 0, 0, 0, 0, 0, 0)
            model.setRoot (r)
            r.parent = None
        InnerModelReader.recursive (root, model, model.root)
        return True

    # need a bit modification for the tags having dimensions
    @staticmethod
    def getClass (model, node, domnode):
        '''Get the class for the corresponding tag element'''

        attr = domnode.attrib
        tag = domnode.tag
        list1 = attr.keys()
        list2 = InnerModelReader.validAttr[tag.lower()]

        if (not set(list1).issubset(set(list2))):
            raise Exception ("%s tag doesn't have valid attributes" % tag)

        if tag.lower() == 'rotation':
            tr = model.newTransform (id=domnode.get('id'), engine=domnode.get('engine', 'static'),
                                     parent=node, tx=0, ty=0, tz=0, rx=float(domnode.get('rx', 0)),
                                     ry=float(domnode.get('ry', 0)), rz=float(domnode.get('rz', 0)),
                                     mass=float(domnode.get('mass', 0)))
            tr.gui_translation = False
            return tr
        elif tag.lower() == 'translation':
            tr = model.newTransform (id=domnode.get('id'), engine=domnode.get('engine', 'static'),
                        parent=node, tx=float(domnode.get('tx', 0)), ty=float(domnode.get('ty', 0)),
                        tz=float(domnode.get('tz', 0)), rx=0, ry=0, rz=0,
                        mass=float(domnode.get('mass', 0)))
            tr.gui_rotation = False
            return tr
        elif tag.lower() == 'transform':
            tr = model.newTransform (id=domnode.get('id'), engine=domnode.get('engine', 'static'),
                                    tx=float(domnode.get('tx', 0)), ty=float(domnode.get('ty', 0)),
                                    tz=float(domnode.get('tz', 0)), rx=float(domnode.get('rx', 0)),
                                    ry=float(domnode.get('ry', 0)), rz=float(domnode.get('rz', 0)),
                                    mass=float(domnode.get('mas', 0)), parent=node)
            return tr
        elif tag.lower() == 'touchsensor':
            ts = model.newTouchSensor (id=domnode.get('id'), stype=domnode.get('type', '0'),
                                    nx=float(domnode.get('nx', 0)), ny=float(domnode.get('ny', 0)),
                    nz=float(domnode.get('nz', 0)), min=float(domnode.get('min', -1*float("inf"))),
                    max=float(domnode.get('max', float("inf"))), port=int(domnode.get('port', 0)),
                    parent=node)
            return ts
        elif tag.lower() == 'joint':
            jr = model.newJoint (id=domnode.get('id'), lx=float(domnode.get('lx', 0)),
                                ly=float(domnode.get('ly', 0)), lz=float(domnode.get('lz', 0)),
                                hx=float(domnode.get('hx', 0)), hy=float(domnode.get('hy', 0)),
                                hz=float(domnode.get('hz', 0)), tx=float(domnode.get('tx', 0)),
                                ty=float(domnode.get('ty', 0)), tz=float(domnode.get('tz', 0)),
                                rx=float(domnode.get('rx', 0)), ry=float(domnode.get('ry', 0)),
                                rz=float(domnode.get('rz', 0)), port=int(domnode.get('port', 0)),
                                min=float(domnode.get('min', -1*float("inf"))), parent=node,
                                max=float(domnode.get('max', float("inf"))),
                                axis=domnode.get('axis', "z"), home=float(domnode.get('home', 0)))
            return jr
        elif tag.lower() == 'prismaticjoint':
            jr = model.newPrismaticJoint (id=domnode.get('id'), port=int(domnode.get('port', 0)),
                                          min=float(domnode.get('min', -1*float("inf"))),
                                          max=float(domnode.get('max', float("inf"))),
                                          position=float(domnode.get('position', 0)),
                                          offset=float(domnode.get('offset', 0)),
                                          axis=domnode.get('axis', "z"),
                                          home=float(domnode.get('home', 0)), parent=node)
            return jr
        elif tag.lower() == 'differentialrobot':
            dr = model.newDifferentialRobot (id=domnode.get('id'), tx=float(domnode.get('tx', 0)),
                                    ty=float(domnode.get('ty', 0)), tz=float(domnode.get('tz', 0)),
                                    rx=float(domnode.get('rx', 0)), ry=float(domnode.get('ry', 0)),
                                    rz=float(domnode.get('rz', 0)), port=int(domnode.get('port', 0)),
                                    noise=float(domnode.get('noise', 0)), parent=node,
                                    collide=(float(domnode.get('collide', 0)) > 0))
            return dr
        elif tag.lower() == 'omnirobot':
            om = model.newOmniRobot (id=domnode.get('id'), tx=float(domnode.get('tx', 0)),
                                     ty=float(domnode.get('ty', 0)), tz=float(domnode.get('tz', 0)),
                                     rx=float(domnode.get('rx', 0)), ry=float(domnode.get('ry', 0)),
                                     rz=float(domnode.get('rz', 0)), port=int(domnode.get('port', 0)),
                                     noise=float(domnode.get('noise', 0)), parent=node,
                                     collide=(float(domnode.get('collide', 0)) > 0))
            return om
        elif tag.lower() == 'camera':
            cam = model.newCamera (id=domnode.get('id'), width=float(domnode.get('width', 0)),
                    height=float(domnode.get('height', 0)), focal=float(domnode.get('focal', 0)))
            return cam
        elif tag.lower() == 'rgbd':
            rgbd = model.newRGBD (id=domnode.get('id'),
                                  width=float(domnode.get('width', 0)),
                                  height=float(domnode.get('height', 0)),
                                  focal=float(domnode.get('focal', 0)),
                                  noise=float(domnode.get('noise', 0)),
                                  port=int(domnode.get('pose', 0)),
                                  ifconfig=domnode.get('ifconfig', ''),
                                  parent=node)
            return rgbd
        elif tag.lower() == 'imu':
            imu = model.newIMU (id=domnode.get('id'), port=int(domnode.get('port', 0)), parent=node)
            return imu
        elif tag.lower() == 'laser':
            laser = model.newLaser (id=domnode.get('id'), port=int(domnode.get('port', 0)),
                                    min=float(domnode.get('min')), max=float(domnode.get('max')),
                                    parent=node, angle=float(domnode.get('angle')),
                                    measures=int(domnode.get('measures')),
                                    ifconfig=domnode.get('ifconfig'))
            return laser
        elif tag.lower() == 'mesh':
            render = domnode.get ('render', 'normal')

            if (render is not 'normal') and (render is not 'wireframe'):
                raise Exception ("rendering mode not valid: %s" % render)

            scale = domnode.get ('scale', '1')
            li = scale.split (',')
            scalex = float(li[0])
            scaley = scalex
            scalez = scaley

            if len(li) == 2:
                scaley = float(li[1])
            elif len(li) == 3:
                scaley = float(li[1]); scalez = float (li[2])
            elif len(li) > 3:
                raise Exception ("too many numbers in scale")

            mesh = model.newMesh (id=domnode.get('id'), path=domnode.get('file'), scalex=scalex,
                                  scaley=scaley, scalez=scalez, render=render,
                                  tx=float(domnode.get('tx', 0)), ty=float(domnode.get('ty', 0)),
                                  tz=float(domnode.get('tz', 0)), rx=float(domnode.get('rx', 0)),
                                  ry=float(domnode.get('ry', 0)), rz=float(domnode.get('rz', 0)),
                                  collidable=(float(domnode.get('collide', 0)) > 0), parent=node)
            return mesh
        elif tag.lower() == 'pointcloud':
            pc = model.newPointCloud (id=domnode.get('id'), parent=node)
            return pc
        elif tag.lower() == 'innermodel':
            raise Exception ("Tag <innermodel> can only be the root tag.")
        elif tag.lower() == 'plane':
            size = domnode.get ('size', '2500')
            li = size.split (',')
            width = float(li[0])/100
            height = width
            depth = height

            if len(li) == 2:
                height = float(li[1])/100
                depth = min (width, height)/100
            elif len(li) == 3:
                height = float(li[1])/100
                depth = float(li[2])/100
            elif len(li) > 3:
                raise Exception ("too many numbers in plane definitions")

            plane = model.newPlane (id=domnode.get('id'), texture=domnode.get('texture', ''),
                                    width=width, height=height, depth=depth,
                                    nx=float(domnode.get('nx', 0)), ny=float(domnode.get('ny', 0)),
                                    nz=float(domnode.get('nz', 0)), px=float(domnode.get('px', 0)),
                                    py=float(domnode.get('py', 0)), pz=float(domnode.get('pz', 0)),
                                    collidable=(float(domnode.get('collide', 0)) > 0), parent=node,
                                    repeat=int(domnode.get('repeat', 1000)))
            return plane
        elif tag.lower() == 'include':
            InnerModelReader.include (attr['path'], model, node)
        elif tag.lower() == 'axes':
            defaultLength = attr['length']
            defaultWidth = attr['width']

            lengths = []
            widths = []

            for _ in range (3):
                lengths.append (200 if defaultLength < 0 else defaultLength)
                widths.append (15 if defaultWidth < 0 else defaultWidth)

            xLength = attr['xlength']
            if (xLength > 0):
                lengths[0] = xLength

            xWidth = attr['xwidth']
            if (xWidth > 0):
                widths[0] = xWidth

            yLength = attr['xlength']
            if (yLength > 0):
                lengths[1] = yLength

            yWidth = attr['xwidth']
            if (yWidth > 0):
                widths[1] = yWidth

            zLength = attr['xlength']
            if (zLength > 0):
                lengths[2] = zLength

            zWidth = attr['xwidth']
            if (zWidth > 0):
                widths[2] = zWidth

            plane = model.newPlane (id=attr['id']+'x', parent=node, texture="#ff0000",
                                    width=widths[0], height=widths[0], depth=lengths[0],
                                    repeat=1, nx=1, ny=0, nz= 0, px=lengths[0]/2, py=0, pz=0,
                                    collidable=False)
            node.addChild (plane)
            node.innerModel = plane.model = model

            plane = model.newPlane (id=attr['id']+'y', parent=node, texture="#00ff00",
                                    width=widths[1], height=lengths[1], depth=widths[1],
                                    repeat=1, nx=1, ny=0, nz= 0, px=0, py=lengths[1]/2, pz=0,
                                    collidable=False)
            node.addChild (plane)
            node.innerModel = plane.model = model

            plane = model.newPlane (id=attr['id']+'z', parent=node, texture="#0000ff",
                                    width=lengths[2], height=widths[2], depth=widths[2],
                                    repeat=1, nx=1, ny=0, nz= 0, px=0, py=0, pz=lengths[2]/2,
                                    collidable=False)
            node.addChild (plane)
            node.innerModel = plane.model = model

            plane = model.newPlane (id=attr['id']+'c', parent=node, texture="#ffffff",
                                    width=widths[0]*1.3, height=widths[1]*1.3, depth=widths[2]*1.3,
                                    repeat=1, nx=1, ny=0, nz= 0, px=0, py=0, pz=0, collidable=False)
            node.addChild (plane)
            node.innerModel = plane.model = model

            return plane
        elif tag.lower() == 'display':
            size = domnode.get ('size', '2500')
            li = size.split (',')
            width = float(li[0])
            height = width
            depth = height/100

            if len(li) == 2:
                height = float(li[1])
                depth = min (width, height)/100
            elif len(li) == 3:
                height = float(li[1])
                depth = float(li[2])
            elif len(li) > 3:
                raise Exception ("too many numbers in plane definitions")

            display = model.newDisplay (id=domnode.get('id'), port=int(domnode.get('port', 0)),
                                        texture=domnode.get('texture', ''), width=width, height=height,
                                        depth=depth, repeat=int(domnode.get('repeat', 1000)),
                                        nx=float(domnode.get('nx', 0)), ny=float(domnode.get('ny', 0)),
                                        nz=float(domnode.get('nz', 0)), px=float(domnode.get('px', 0)),
                                        py=float(domnode.get('py', 0)), pz=float(domnode.get('pz', 0)),
                                        collidable=(float(domnode.get('collide', 0)) > 0), parent=node)
            return display
        else:
            print ("%s is not a valid tag name", tag)
            return None

    @staticmethod
    def isValidTag (tag):
        '''To find out if a tag is valid or not'''

        if ((tag == 'innermodel') or
            (tag == 'transform') or
            (tag == 'rotation') or
            (tag == 'translation') or
            (tag == 'joint') or
            (tag == 'touchsensor') or
            (tag == 'prismaticjoint') or
            (tag == 'differentialrobot') or
            (tag == 'omnirobot')):
            return True
        else:
            return False

    @staticmethod
    def recursive (parentDomNode: 'ET', model: 'InnerModel', imNode: 'InnerModelNode'):
        '''Recursively read the xml doc'''

        if (not InnerModelReader.isValidTag (parentDomNode.tag.lower()) and len(parentDomNode) > 0):
            print (parentDomNode.tag)
            raise Exception ("Not a valid tag")
        for child in parentDomNode:
            node = InnerModelReader.getClass (model, imNode, child)
            imNode.addChild (node)
            node.innerModel = model
            InnerModelReader.recursive (child, model, node)

    def getValidNodeAttributes (self):
        '''Returns the valid attributes for all the tag elements'''

        return InnerModelReader.validAttr
