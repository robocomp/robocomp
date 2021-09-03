from PySide2.QtCore import Signal
from PySide2.QtWidgets import QTreeWidget, QTreeWidgetItem


class TreeViewer(QTreeWidget):
    node_check_state_changed_signal = Signal(int, int, str, QTreeWidgetItem)

    def __init__(self, G):
        super(TreeViewer, self).__init__()
        self.G = G
        self.types_map = {}
        self.tree_map = {}
        self.attributes_map = {}

    def getGraph(self):
        return self.G

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

    def reload(self, widget):
        pass

    # PRIVATE
    def createGraph(self, ):
        pass

    def create_attribute_widgets(self, parent, node):
        pass

    def create_attribute_widget(self, parent, node, key, value):
        pass

    def update_attribute_widgets(self, node):
        pass
