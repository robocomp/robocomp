import sys

from PySide2.QtCore import QObject, QTimer, QElapsedTimer, Signal, QSettings
from PySide2.QtGui import Qt
from PySide2.QtWidgets import QWidget, QMenu, QMainWindow, QApplication, QAction, QDockWidget, QFileDialog, QPushButton

from viewers.qscene_2d_viewer.qscene_2d_viewer import QScene2dViewer
from viewers.tree_viewer.tree_viewer import TreeViewer


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
        self.g = G
        self.window = window
        self.view_menu = QMenu()
        self.file_menu = QMenu()
        self.forces_menu = QMenu()
        self.main_widget = window
        self.docks = {}
        self.widgets = {}
        self.widgets_by_type = {}

        available_geometry = QApplication.desktop().availableGeometry()
        window.move((available_geometry.width() - window.width()) / 2,
                    (available_geometry.height() - window.height()) / 2)
        self.__initialize_file_menu()
        viewMenu = window.menuBar().addMenu(window.tr("&View"))
        forcesMenu = window.menuBar().addMenu(window.tr("&Forces"))
        actionsMenu = window.menuBar().addMenu(window.tr("&Actions"))
        restart_action = actionsMenu.addAction("Restart")

        self.__initialize_views(options, main)
        self.alive_timer.start()
        self.timer.start(500)
        # self.init()  #intialize processor number
        # connect(timer, SIGNAL(timeout()), self, SLOT(compute()))

    def __del__(self):
        settings = QSettings("RoboComp", "DSR")
        settings.beginGroup("MainWindow")
        settings.setValue("size", self.window.size())
        settings.setValue("pos", self.window.pos())
        settings.endGroup()

    def get_widget_by_type(self, widget_type) -> QWidget:
        if widget_type in self.widgets_by_type:
            return self.widgets_by_type[widget_type].widget
        return None

    def get_widget_by_name(self, name) -> QWidget:
        if name in self.widgets:
            return self.widgets[name].widget
        return None

    def add_custom_widget_to_dock(self, name, custom_view):
        widget_c = WidgetContainer()
        widget_c.name = name
        widget_c.type = View.none
        widget_c.widget = custom_view
        self.widgets[name] = widget_c
        self.__create_dock_and_menu(name, custom_view)
        # Tabification of current docks
        previous = None
        for dock_name, dock in self.docks.items():
            if previous and previous != dock:
                self.window.tabifyDockWidget(previous, self.docks[name])
                break
            previous = dock
        self.docks[name].raise_()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close_window_signal.emit()

    # SLOTS
    def save_graph_slot(self, state):
        self.save_graph_signal.emit()

    def restart_app(self, state):
        pass

    def switch_view(self, state, container):
        widget = container.widget
        dock = container.dock
        if state:
            widget.blockSignals(True)
            dock.hide()
        else:
            widget.blockSignals(False)
            self.reset_viewer.emit(widget)
            dock.show()
            dock.raise_()

    def compute(self):
        pass

    def __create_dock_and_menu(self, name, view):
        # TODO: Check if name exists in docks
        if name in self.docks:
            dock_widget = self.docks[name]
            self.window.removeDockWidget(dock_widget)
        else:
            dock_widget = QDockWidget(name)
            new_action = QAction(name, self)
            new_action.setStatusTip("Create a new file")
            new_action.setCheckable(True)
            new_action.setChecked(True)
            new_action.triggered.connect(lambda state: self.switch_view(state, self.widgets[name]))
            self.view_menu.addAction(new_action)
            self.docks[name] = dock_widget
            self.widgets[name].dock = dock_widget
        dock_widget.setWidget(view)
        dock_widget.setAllowedAreas(Qt.AllDockWidgetAreas)
        self.window.addDockWidget(Qt.RightDockWidgetArea, dock_widget)
        dock_widget.raise_()

    def __initialize_views(self, options, central):
        # Create docks view and main widget
        valid_options = [(View.graph, "Graph"), (View.tree, "Tree"), (View.osg, "3D"), (View.scene, "2D")]

        # Creation of docks and mainwidget
        for widget_type, widget_name in valid_options:
            if widget_type == central and central != View.none:
                viewer = self.__create_widget(widget_type)
                self.window.setCentralWidget(viewer)
                widget_c = WidgetContainer()
                widget_c.widget = viewer
                widget_c.name = widget_name
                widget_c.type = widget_type
                self.widgets[widget_name] = widget_c
                self.widgets_by_type[widget_type] = widget_c
                self.main_widget = viewer
            elif options & widget_type:
                viewer = self.__create_widget(widget_type)
                widget_c = WidgetContainer()
                widget_c.widget = viewer
                widget_c.name = widget_name
                widget_c.type = widget_type
                self.widgets[widget_name] = widget_c
                self.widgets_by_type[widget_type] = widget_c
                self.__create_dock_and_menu(widget_name, viewer)
        if View.graph in self.widgets_by_type:
            new_action = QAction("Animation", self)
            new_action.setStatusTip("Toggle animation")
            new_action.setCheckable(True)
            new_action.setChecked(False)
            self.forces_menu.addAction(new_action)
            new_action.triggered.connect(lambda: self.widgets_by_type[View.graph].widget.toggle_animation(True))

        # Tabification of current docks
        previous = None
        for dock_name, dock_widget in self.docks.items():
            if previous:
                self.window.tabifyDockWidget(previous, dock_widget)
            previous = dock_widget

        # Connection of tree to graph signals
        if "Tree" in self.docks:
            if self.main_widget:
                graph_widget = self.main_widget
                if graph_widget:
                    tree_widget = self.docks["Tree"].widget()
                    tree_widget.node_check_state_changed_signal.connect(
                        lambda node_id: graph_widget.hide_show_node_SLOT(node_id, 2)
                    )
        if len(self.docks) > 0 or central != None:
            self.window.show()
        else:
            self.window.showMinimized()

    def __initialize_file_menu(self):
        file_menu = self.window.menuBar().addMenu(self.window.tr("&File"))
        file_submenu = file_menu.addMenu("Save")
        save_action = QAction("Save", self)
        file_submenu.addAction(save_action)
        rgbd = QAction("RGBD", self)
        rgbd.setCheckable(True)
        rgbd.setChecked(False)
        file_submenu.addAction(rgbd)
        laser = QAction("Laser", self)
        laser.setCheckable(True)
        laser.setChecked(False)
        file_submenu.addAction(laser)
        # save_action
        save_action.triggered.connect(lambda: self.__save_json_file(rgbd, laser))

    def __save_json_file(self, rgbd, laser):
        file_name = QFileDialog.getSaveFileName(None, "Save file",
                                                "/home/robocomp/robocomp/components/dsr-graph/etc",
                                                "JSON Files (*.json)",
                                                None,
                                                QFileDialog.Option.DontUseNativeDialog)
        skip_content = []
        if not rgbd.isChecked():
            skip_content.push_back("rgbd")
        if not laser.isChecked():
            skip_content.push_back("laser")
        self.g.write_to_json_file(file_name.toStdString(), skip_content)
        print("File saved")

    def __create_widget(self, widget_type):
        widget_view = None
        if widget_type == View.graph:
            widget_view = GraphViewer(self.g)
        elif widget_type == View.osg:
            widget_view = OSG3dViewer(self.g, 1, 1)
        elif widget_type == View.tree:
            widget_view = TreeViewer(self.g)
        elif widget_type == View.scene:
            widget_view = QScene2dViewer(self.g)
        elif widget_type == View.none:
            widget_view = None
        # self.reset_viewer.connect(self.reload)
        return widget_view


if __name__ == '__main__':
    sys.path.append('/opt/robocomp/lib')
    app = QApplication(sys.argv)
    from pydsr import *

    g = DSRGraph(0, "pythonAgent", 111)
    node = g.get_node("world")
    main_window = QMainWindow()
    print(node)
    current_opts = View.tree
    ui = DSRViewer(main_window, g, current_opts)
    custom_widget = QPushButton("Custom Widget")
    ui.add_custom_widget_to_dock("Custom", custom_widget)
    print("Setup")
    main_window.show()
    app.exec_()
