import sys
import math
import pathlib

from PyQt5 import QtGui, QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QWidget, QApplication

from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap


class Obstacle:
    def __init__(self, vertices):
        self.vertices = vertices

    def is_inside(self, point):
        x, y = point
        num_vertices = len(self.vertices)
        inside = False
        p1x, p1y = self.vertices[0]
        for i in range(num_vertices + 1):
            p2x, p2y = self.vertices[i % num_vertices]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside


class MyCartWindow(QWidget):

    def __init__(self, _compound_sys, _img='my4.png'):
        super(MyCartWindow, self).__init__()
        self.compound_system = _compound_sys
        self.image = _img
        self.obstacles=None
        self.initUI()
        

    def initUI(self):
        layout = QVBoxLayout(self)

        # Carica l'immagine
        pixmap = QPixmap(self.image)

        # Crea un QLabel per visualizzare l'immagine
        label = QLabel(self)
        label.setPixmap(pixmap)  # Imposta la pixmap nel QLabel

        # Aggiungi il QLabel al layout
        layout.addWidget(label)

        self.setLayout(layout)

        self.setLayout(layout)

        self.setGeometry(0, 0, 1000, 600)
        self.setWindowTitle('Robot 2D Simulator')
        self.show()

        current_path = pathlib.Path(__file__).parent.resolve()
        image = str(current_path) + '/../icons/' + self.image

        self.robot_pic = QtGui.QPixmap(image)

        self.delta_t = 1e-4  # 0.1ms of time-tick

        self._timer_painter = QtCore.QTimer(self)
        self._timer_painter.start(int(self.delta_t * 1000))
        self._timer_painter.timeout.connect(self.go)

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
        self.obstacles_polygons = []  # Converti gli ostacoli in QPolygonF

        if self.obstacles is not None:
            for obstacle in self.obstacles:
                p = QtGui.QPolygonF()
                for point in obstacle:
                    p.append(QtCore.QPointF(point[0], point[1]))
                self.obstacles_polygons.append(p)


    def go(self):
        if not (self.compound_system.step()):
            self._timer_painter.stop()
        self.update()  # repaint window

    def paintEvent(self, event):
        qp = QtGui.QPainter()
        qp.begin(self)
        qp.setPen(QtGui.QColor(255, 255, 255))
        qp.setBrush(QtGui.QColor(255, 255, 255))
        qp.drawRect(event.rect())

        (x, y, theta) = self.compound_system.get_pose()

        qp.setPen(Qt.black)

        qp.drawText(910, 20, "X  = %6.3f m" % (x))
        qp.drawText(910, 40, "Y  = %6.3f m" % (y))
        qp.drawText(910, 60, "Th = %6.3f deg" % (math.degrees(theta)))
        qp.drawText(910, 80, "T  = %6.3f s" % (self.compound_system.t))

        # Disegno degli ostacoli
        qp.setBrush(QtGui.QColor('#023047'))
        for polygon in self.obstacles_polygons:
            qp.drawPolygon(polygon)

        # if self.obstacles is not None:
        #     for obstacle in self.obstacles:
        #         p = QtGui.QPolygonF()
        #         for point in obstacle:
        #             p.append(QtCore.QPointF(point[0], point[1]))
        #         qp.setBrush(QtGui.QColor('#023047'))
        #         qp.drawPolygon(p)


        # #Disegno gli ostacoli
        # p1 = QtGui.QPolygonF()
        # p1.append(QtCore.QPointF(190, 260))  
        # p1.append(QtCore.QPointF(100, 350))  
        # p1.append(QtCore.QPointF(250, 400))  
        # qp.setBrush(QtGui.QColor('#023047'))
        # qp.drawPolygon(p1)


        # p2 = QtGui.QPolygonF()
        # p2.append(QtCore.QPointF(490, 200))  
        # p2.append(QtCore.QPointF(600, 130))
        # p2.append(QtCore.QPointF(800, 300))
        # p2.append(QtCore.QPointF(700, 500))

        # qp.setBrush(QtGui.QColor('#023047'))
        # qp.drawPolygon(p2)

        
        # p3 = QtGui.QPolygonF()
        # p3.append(QtCore.QPointF(30, 60))  
        # p3.append(QtCore.QPointF(40, 80))  
        # p3.append(QtCore.QPointF(50, 60))  
        # p3.append(QtCore.QPointF(40, 50))

        # qp.setBrush(QtGui.QColor('#023047')) 
        # qp.drawPolygon(p3)

        s = self.robot_pic.size()/2
        x_pos = int(10 + x * 1000 - s.width() / 2)
        y_pos = int(500 - y * 1000 - s.height() / 2)

        t = QtGui.QTransform()
        t.translate(x_pos + s.width() / 2, y_pos + s.height() / 2)
        t.rotate(-math.degrees(theta))
        t.translate(-(x_pos + s.width() / 2), -(y_pos + s.height() / 2))

        qp.setTransform(t)
        qp.drawPixmap(x_pos, y_pos, self.robot_pic)

        qp.end()


def main():
    app = QApplication(sys.argv)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

