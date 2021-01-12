from PySide2.QtCore import QObject, QTimer, QElapsedTimer, Signal
from PySide2.QtWidgets import QWidget, QMenu


class View:
    none = -1
    graph = (1 << 0)
    osg = (1 << 1)
    scene = (1 << 2)
    tree = (1 << 3)


class WidgetContainer:
    def __init__(self):
        self.name = ""
        self.widget_type = View()
        self.widget = None
        self.dock = None


class DSRViewer(QObject):
    save_graph_signal = Signal()
    close_window_signal = Signal()
    reset_viewer = Signal(QWidget)

    def __init__(self, window, G, options, main=None):
        super().__init__()
        self.timer = QTimer()
        self.alive_timer = QElapsedTimer()
        self.G = G
        self.window = window
        self.view_menu = QMenu()
        self.file_menu = QMenu()
        self.forces_menu = QMenu()
        self.main_widget = None

    def item_moved(self):
        pass

    def create_graph(self):
        pass

    def get_widget_by_type(self, widget_type) -> QWidget:
        return QWidget()

    def get_widget_by_name(self, name) -> QWidget:
        return QWidget()

    def add_custom_widget_to_dock(self, name, view):
        pass

    def keyPressEvent(self, event):
        pass

    # SLOTS
    def saveGraphSLOT(self, state):
        pass

    def restart_app(self, state):
        pass

    def switch_view(self, state, container):
        pass

    def compute(self):
        pass

    def __create_dock_and_menu(self, name, view):
        pass

    def __initialize_views(self, options, main):
        pass

    def __initialize_file_menu(self):
        pass

    def __create_widget(self, widget_type) -> QWidget:
        pass
