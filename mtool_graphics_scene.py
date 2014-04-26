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


class AxesItem(QtGui.QGraphicsItem):
    def __init__(self, parent=None):
        super(AxesItem, self).__init__(parent)
        self.x_scale = 100.0
        self.y_scale = 100.0
        self.x_suffix = ''
        self.y_suffix = ''

        self.x_format = '{0:+.0f}{suffix}'
        self.y_format = '{0:+.0f}{suffix}'

    def boundingRect(self):
        return QtCore.QRectF(-1.0, -1.0, 2.0, 2.0)

    def paint(self, painter, option, widget):
        # Draw the major axes.
        painter.drawLine(QtCore.QPointF(-1.0, 0.0), QtCore.QPointF(1.0, 0.0))
        painter.drawLine(QtCore.QPointF(0.0, -1.0), QtCore.QPointF(0.0, 1.0))

        transform = painter.transform()
        painter.resetTransform()

        def line(x1, y1, dx, dy):
            point = transform.map(QtCore.QPointF(x1, y1))
            delta = QtCore.QPointF(dx, dy)
            painter.drawLine(point + delta, point - delta)

        # Next draw the ticks.
        d = 4
        for i in range(1, 11):
            line(0.1 * i, 0, 0, d)
            line(-0.1 * i, 0, 0, d)
            line(0, 0.1 * i, d, 0)
            line(0, -0.1 * i, d, 0)

        width = 200
        # Finally, draw the labels.
        point = transform.map(QtCore.QPointF(-1.0, 0.))
        rect = QtCore.QRectF(point - QtCore.QPointF(0, 40),
                             point + QtCore.QPointF(width, -d))
        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignLeft,
            self.x_format.format(-self.x_scale, suffix=self.x_suffix))


        point = transform.map(QtCore.QPointF(1.0, 0.))
        rect = QtCore.QRectF(point - QtCore.QPointF(width, 40),
                             point - QtCore.QPointF(0, d))

        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignRight,
            self.x_format.format(self.x_scale, suffix=self.x_suffix))

        point = transform.map(QtCore.QPointF(0, 1.0))
        rect = QtCore.QRectF(point + QtCore.QPointF(d, 0),
                             point + QtCore.QPointF(width, 40))
        painter.drawText(
            rect,
            QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
            self.y_format.format(self.y_scale, suffix=self.y_suffix))

        point = transform.map(QtCore.QPointF(0, -1.0))
        rect = QtCore.QRectF(point + QtCore.QPointF(d, -40),
                             point + QtCore.QPointF(width, 0))
        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignLeft,
            self.y_format.format(-self.y_scale, suffix=self.y_suffix))
