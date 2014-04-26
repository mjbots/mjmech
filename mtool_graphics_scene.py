# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

class GraphicsScene(QtGui.QGraphicsScene):
    sceneMouseMoveEvent = QtCore.Signal(QtCore.QPointF)
    sceneMousePressEvent = QtCore.Signal(QtCore.QPointF)
    sceneMouseReleaseEvent = QtCore.Signal()

    def __init__(self, parent=None):
        super(GraphicsScene, self).__init__(parent)

    def mouseMoveEvent(self, event):
        if event.buttons() & QtCore.Qt.LeftButton:
            self.sceneMouseMoveEvent.emit(event.scenePos())

    def mousePressEvent(self, event):
        if event.button() & QtCore.Qt.LeftButton:
            self.sceneMousePressEvent.emit(event.scenePos())

    def mouseReleaseEvent(self, event):
        if event.button() & QtCore.Qt.LeftButton:
            self.sceneMouseReleaseEvent.emit()
