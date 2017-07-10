

""" This class contain Values about the network representation and simulation """
from PyQt4 import QtGui
from PyQt4.QtCore import Qt
from widgets import menus


class NetworkGraphicValues:  
    def __init__(self):
        self.xstrech = 1
        self.ystrech = 1
        self.spring_length = 100
        self.field_force_multiplier = 20000
        self.hookes_constant = .07
        self.time_elapsed2 = 3
        self.roza = .8
        self.Groups = []
        
class ComponentTree(QtGui.QGraphicsView):  ##The widget on which we are going to draw the graphtree
    def __init__(self, parent, mainclass):
        self.viewportAnchor = QtGui.QGraphicsView.AnchorUnderMouse
        QtGui.QGraphicsView.__init__(self, parent)
        self.connection_building_status = False
        self.mainclass = mainclass  # This object is the mainClass from rcmanager Module
        self.CompoPopUpMenu = menus.ComponentMenu(self.mainclass)
        self.BackPopUpMenu = menus.BackgroundMenu(self.mainclass)

        self.lastPosition = None
        self.leftMouseButtonClicked = False

    # self.ctrlButtonClicked=False

    def wheelEvent(self, wheel):
        QtGui.QGraphicsView.wheelEvent(self, wheel)
        temp = self.mainclass.currentZoom
        temp += (wheel.delta() / 120)
        self.mainclass.verticalSlider.setValue(temp)
        self.mainclass.graph_zoom()

    def contextMenuEvent(self, event):  ##It will select what kind of context menu should be displayed
        # if self.ctrlButtonClicked:
        #	return

        if self.leftMouseButtonClicked:
            return

        GloPos = event.globalPos()
        pos = event.pos()
        item = self.itemAt(pos)

        if isinstance(item, VisualNode):
            self.CompoPopUpMenu.set_component(item)
            self.CompoPopUpMenu.popup(GloPos)
        else:
            self.BackPopUpMenu.setPos(pos)
            self.BackPopUpMenu.popup(GloPos)

    """
    def keyPressEvent(self, event):
        if event.key()==Qt.Key_Control:
            self.ctrlButtonClicked=True
    """

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.leftMouseButtonClicked = True

            print "Left mouse button pressed"

        if event.button() == Qt.RightButton and self.leftMouseButtonClicked:
            self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
            self.lastPosition = event.pos()

            print "Right mouse button pressed, entering pan mode"

            return

        if event.button() == Qt.MidButton:
            self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
            self.lastPosition = event.pos()

            print "Middle mouse button pressed, entering pan mode"

            return

        """	
        if event.button()==Qt.RightButton and self.ctrlButtonClicked:
            self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
            self.lastPosition=event.pos()
            return
        """

        if self.connection_building_status == True:
            pos = event.pos()
            item = self.itemAt(pos)
            if isinstance(item, VisualNode):
                self.mainclass.connectionBuilder.setEnd(item.parent)
            self.connection_building_status = False

        if self.connection_building_status == False:
            QtGui.QGraphicsView.mousePressEvent(self, event)

    def mouseMoveEvent(self, event):
        if self.dragMode() == QtGui.QGraphicsView.ScrollHandDrag:
            self.currentPosition = event.pos()

            print "panning, init pos:", self.lastPosition.x(), self.lastPosition.y(), "final pos:", self.currentPosition.x(), self.currentPosition.y()

            dx = self.currentPosition.x() - self.lastPosition.x()
            dy = self.currentPosition.y() - self.lastPosition.y()
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - dy)
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - dx)
            self.lastPosition = self.currentPosition

        QtGui.QGraphicsView.mouseMoveEvent(self, event)

    """
    def keyReleaseEvent(self, event):
        if event.key()==Qt.Key_Control:
            self.ctrlButtonClicked=False
            self.setDragMode(QtGui.QGraphicsView.NoDrag)
    """

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.leftMouseButtonClicked = False
            self.setDragMode(QtGui.QGraphicsView.NoDrag)

            print "Left mouse button released, leaving pan mode"

        if event.button() == Qt.RightButton:
            self.setDragMode(QtGui.QGraphicsView.NoDrag)

            print "Right mouse button released, leaving pan mode"

        if event.button() == Qt.MidButton:
            self.setDragMode(QtGui.QGraphicsView.NoDrag)

            print "Middle mouse button released, leaving pan mode"

        QtGui.QGraphicsView.mouseReleaseEvent(self, event)

        ##
