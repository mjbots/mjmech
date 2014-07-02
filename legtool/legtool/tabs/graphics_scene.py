# Copyright 2014 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
    def __init__(self, parent=None, true_scale=False, grid_skip=0):
        QtGui.QGraphicsItem.__init__(self, parent)

        self.x_scale = 100.0
        self.y_scale = 100.0
        self.x_suffix = ''
        self.y_suffix = ''
        self.true_scale = true_scale
        self.grid_skip = grid_skip

        self.x_format = '{0:+.0f}{suffix}'
        self.y_format = '{0:+.0f}{suffix}'

    def boundingRect(self):
        xscale = 1.0 if not self.true_scale else self.x_scale
        yscale = 1.0 if not self.true_scale else self.y_scale
        return QtCore.QRectF(-xscale, -yscale, 2 * xscale, 2 * yscale)

    def paint(self, painter, option, widget):
        xscale = 1.0 if not self.true_scale else self.x_scale
        yscale = 1.0 if not self.true_scale else self.y_scale
        # Draw the major axes.
        painter.drawLine(QtCore.QPointF(-xscale, 0.0),
                         QtCore.QPointF(xscale, 0.0))
        painter.drawLine(QtCore.QPointF(0.0, -yscale),
                         QtCore.QPointF(0.0, yscale))

        pen = painter.pen()
        painter.setPen(QtGui.QPen(QtGui.QColor(200, 200, 200)))
        for i in range(1, 11):
            if self.grid_skip != 0 and (i % self.grid_skip) == 0:
                painter.drawLine(QtCore.QPointF(-xscale, 0.1 * i * yscale),
                                 QtCore.QPointF(xscale, 0.1 * i * yscale))
                painter.drawLine(QtCore.QPointF(-xscale, -0.1 * i * yscale),
                                 QtCore.QPointF(xscale, -0.1 * i * yscale))
                painter.drawLine(QtCore.QPointF(-0.1 * i * xscale, -yscale),
                                 QtCore.QPointF(-0.1 * i * xscale, yscale))
                painter.drawLine(QtCore.QPointF(0.1 * i * xscale, -yscale),
                                 QtCore.QPointF(0.1 * i * xscale, yscale))

        painter.setPen(pen)
        transform = painter.transform()
        painter.resetTransform()

        def line(x1, y1, dx, dy):
            point = transform.map(QtCore.QPointF(x1, y1))
            delta = QtCore.QPointF(dx, dy)
            painter.drawLine(point + delta, point - delta)

        # Next draw the ticks.
        d = 4
        for i in range(1, 11):
            line(0.1 * xscale * i, 0, 0, d)
            line(-0.1 * xscale * i, 0, 0, d)

            line(0, 0.1 * yscale * i, d, 0)
            line(0, -0.1 * yscale * i, d, 0)

        width = 200
        # Finally, draw the labels.
        point = transform.map(QtCore.QPointF(-xscale, 0.))
        rect = QtCore.QRectF(point - QtCore.QPointF(0, 40),
                             point + QtCore.QPointF(width, -d))
        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignLeft,
            self.x_format.format(-self.x_scale, suffix=self.x_suffix))


        point = transform.map(QtCore.QPointF(xscale, 0.))
        rect = QtCore.QRectF(point - QtCore.QPointF(width, 40),
                             point - QtCore.QPointF(0, d))

        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignRight,
            self.x_format.format(self.x_scale, suffix=self.x_suffix))

        point = transform.map(QtCore.QPointF(0, yscale))
        rect = QtCore.QRectF(point + QtCore.QPointF(d, 0),
                             point + QtCore.QPointF(width, 40))
        painter.drawText(
            rect,
            QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft,
            self.y_format.format(self.y_scale, suffix=self.y_suffix))

        point = transform.map(QtCore.QPointF(0, -yscale))
        rect = QtCore.QRectF(point + QtCore.QPointF(d, -40),
                             point + QtCore.QPointF(width, 0))
        painter.drawText(
            rect,
            QtCore.Qt.AlignBottom | QtCore.Qt.AlignLeft,
            self.y_format.format(-self.y_scale, suffix=self.y_suffix))
