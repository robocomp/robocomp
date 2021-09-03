
from PySide2.QtCore import Signal
from PySide2.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsRectItem
from PySide2.QtGui import QColor, QPen, QBrush

from viewers._abstract_graphic_view import AbstractGraphicViewer

# constants
ROBOT_LENGTH = 400


class QScene2dViewer(AbstractGraphicViewer):
    drawaxis = False
    axis_center = None
    axis_x = None
    axis_y = None
    G = None

    def __init__(self, G):
        super().__init__()
        self.G = G
#        self.setMinimunSize(400, 400)
        self.scale(1, -1)
        # AXIS
        self.axis_center = QGraphicsRectItem(-100, -100, 200, 200)
        self.axis_center.setPen(QPen(QColor("black")))
        self.axis_center.setBrush(QBrush(QColor("black")))
        self.axis_center.setZValue(5000)
        self.axis_x = QGraphicsRectItem(0, 0, 1000, 30)
        self.axis_x.setPen(QPen(QColor("red")))
        self.axis_x.setBrush(QBrush(QColor("red")))
        self.axis_x.setZValue(5000)
        self.axis_y = QGraphicsRectItem(0, 0, 30, 1000)
        self.axis_y.setPen(QPen(QColor("blue")))
        self.axis_y.setBrush(QBrush(QColor("blue")))
        self.axis_y.setZValue(5000)


        self.set_draw_axis(True)
        self.draw_axis()



    def create_graph(self):
        pass

    def set_draw_axis(self, visible):
        self.drawaxis = visible

    def draw_axis(self):
        if self.drawaxis:
            self.scene.addItem(self.axis_center)
            self.scene.addItem(self.axis_x)
            self.scene.addItem(self.axis_y)
        else:
            self.scene.removeItem(self.axis_center)
            self.scene.removeItem(self.axis_x)
            self.scene.removeItem(self.axis_y)



    # SLOTS
    def add_or_assign_node_slot(self, node_id, node_type, name=""):
        pass

    def add_or_assign_node_slot(self, node_id, node):
        pass

    def add_or_assign_edge_slot(self, from_node, to_node, node_type):
        pass

    def del_edge_slot(self, from_node, to_node, edge_tag):
        pass

    def del_node_slot(self, node_id):
        pass

    def node_change_slot(self, value, node_id, node_type, parent=None):
        pass

    def category_change_slot(self, value, parent=None):
        pass
