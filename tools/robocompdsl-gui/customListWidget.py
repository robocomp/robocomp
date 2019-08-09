from PySide2.QtWidgets import QListWidget
from PySide2.QtCore import Signal, Slot, Qt
from PySide2 import QtCore


class CustomListWidget(QListWidget):
    customItemSelection = Signal()

    def __init__(self, parent=None):
        super(CustomListWidget, self).__init__(parent)
        self.itemList = []
        self.setMinimumSize(QtCore.QSize(160, 0))
        self.setMaximumSize(QtCore.QSize(245, 16777215))

    def mousePressEvent(self, event):
        super(CustomListWidget, self).mousePressEvent(event)
        item = self.itemAt(event.pos())
        if item:
            text = item.text().split(":")[0]
            # check button clicked
            if event.button() == Qt.LeftButton:
                if (event.modifiers() == Qt.ShiftModifier) or (event.modifiers() == Qt.ControlModifier):
                    self.itemList.append(text)
                else:
                    count = self.itemList.count(text)
                    self.clearItems()
                    for c in range(count + 1):
                        self.itemList.append(text)
            elif event.button() == Qt.RightButton:
                if text in self.itemList:
                    self.itemList.remove(text)

            # update list text
            count = self.itemList.count(text)
            self.itemAt(event.pos()).setSelected(count)
            if count:
                self.itemAt(event.pos()).setText(text + ":" + str(count))
            else:
                # self.itemAt(event.pos()).setPlainText(text)
                self.itemAt(event.pos()).setText(text)

            self.customItemSelection.emit()
        else:
            self.clearItems()

    def clearItems(self):
        self.itemList = []
        for pos in range(self.count()):
            self.item(pos).setText(self.item(pos).text().split(":")[0])

    # return our custom selected item list
    def customItemList(self):
        return self.itemList

    # just for testing
    @Slot()
    def print(self):
        print("Selected items\n", self.itemList)
