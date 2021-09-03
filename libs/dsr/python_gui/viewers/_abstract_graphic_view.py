from abc import ABCMeta, ABC

from PySide2.QtCore import Qt, QEventLoop
from PySide2.QtGui import QPainter, QCursor
from PySide2.QtWidgets import QGraphicsView, QGraphicsScene, QWidget, QApplication


class QABCMeta(ABCMeta, type(QWidget)):
    """Create a meta class that combines ABC and the Qt meta class"""
    pass


class TcWidget(ABC, metaclass=QABCMeta):
    """Abstract class, to be multi-inherited together with a Qt item"""
    pass


class AbstractGraphicViewer(QGraphicsView, TcWidget):
    def __init__(self, parent=None):
        super(AbstractGraphicViewer, self).__init__(parent)
        self.m_scaleX = 0
        self.m_scaleY = 0
        self._pan = False
        self._panStartX = 0
        self._panStartY = 0
        self.scene = QGraphicsScene()
        self.scene.setItemIndexMethod(QGraphicsScene.NoIndex)
        self.scene.setSceneRect(-2000, -2000, 4000, 4000)
        self.setScene(self.scene)
        self.setCacheMode(QGraphicsView.CacheBackground)
        self.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)
        self.setRenderHint(QPainter.Antialiasing)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setMinimumSize(400, 400)
        self.adjustSize()
        self.setMouseTracking(True)
        self.viewport().setMouseTracking(True)

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            factor = 1.1
        else:
            factor = 0.9
        view_pos = event.pos()
        scene_pos = self.mapToScene(view_pos)
        self.centerOn(scene_pos)
        self.scale(factor, factor)
        delta = self.mapToScene(view_pos) - self.mapToScene(self.viewport().rect().center())
        self.centerOn(scene_pos - delta)

    def resizeEvent(self, event):
        super(AbstractGraphicViewer, self).resizeEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan:
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - (event.x() - self._panStartX))
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - (event.y() - self._panStartY))
            self._panStartX = event.x()
            self._panStartY = event.y()
            event.accept()
        super(AbstractGraphicViewer, self).mouseMoveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._pan = True
            self._panStartX = event.x()
            self._panStartY = event.y()
            self.setCursor(QCursor.ClosedHandCursor)
            event.accept()
            return
        super(AbstractGraphicViewer, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._pan = False
            self.setCursor(QCursor.ArrowCursor)
            event.accept()
        super(AbstractGraphicViewer, self).mouseReleaseEvent(event)

    def showEvent(self, event):
        super(AbstractGraphicViewer, self).showEvent(event)
        adjusted = self.scene.itemsBoundingRect().adjusted(-100, -100, 100, 100)
        self.scene.setSceneRect(adjusted)
        # FitInView is called two times because of this bug: https://bugreports.qt.io/browse/QTBUG-1047
        update_state = self.updatesEnabled()
        self.setUpdatesEnabled(False)
        self.fitInView(adjusted, Qt.KeepAspectRatio)
        QApplication.processEvents(QEventLoop.ExcludeUserInputEvents)
        self.fitInView(adjusted, Qt.KeepAspectRatio)
        self.setUpdatesEnabled(update_state)
