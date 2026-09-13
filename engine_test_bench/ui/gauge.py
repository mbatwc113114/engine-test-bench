from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import QBrush, QFont, QPainter, QPen
from PyQt5.QtWidgets import QWidget
import math


class Gauge(QWidget):
    def __init__(self, gauge_id, min_value=0, max_value=100, unit="", parent=None):
        super().__init__(parent)
        self.id = gauge_id
        self.min_value = float(min_value)
        self.max_value = float(max_value)
        self.unit = unit
        self._value = self.min_value
        self.start_angle = -135.0
        self.end_angle = 135.0
        self.needle_length_ratio = 0.34
        self.needle_width = 5
        self.needle_color = Qt.red
        self.hub_radius_ratio = 0.055
        self.value_rect = (0.25, 0.63, 0.50, 0.20)
        self.setAttribute(Qt.WA_TranslucentBackground, True)
        self.setMinimumSize(120, 120)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, new_value):
        self.setValue(new_value)

    def setValue(self, value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return

        value = max(self.min_value, min(self.max_value, value))
        self._value = value
        self.update()

    def getValue(self):
        return self._value

    def setRange(self, minimum, maximum):
        minimum = float(minimum)
        maximum = float(maximum)
        if maximum <= minimum:
            raise ValueError("Maximum must be greater than minimum")
        self.min_value = minimum
        self.max_value = maximum
        self.setValue(self._value)

    def setUnit(self, unit):
        self.unit = unit
        self.update()

    def valueToAngle(self):
        if self.max_value == self.min_value:
            return self.start_angle

        percentage = max(0.0, min(1.0, (self._value - self.min_value) / (self.max_value - self.min_value)))
        return self.start_angle + percentage * (self.end_angle - self.start_angle)

    def point_from_angle(self, center_x, center_y, radius, angle):
        radians = math.radians(angle)
        x = center_x + radius * math.sin(radians)
        y = center_y - radius * math.cos(radians)
        return QPointF(x, y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)

        width = self.width()
        height = self.height()
        size = min(width, height)
        center_x = width / 2
        center_y = height / 2

        angle = self.valueToAngle()
        needle_length = size * self.needle_length_ratio
        needle_start = self.point_from_angle(center_x, center_y, -size * 0.03, angle)
        needle_end = self.point_from_angle(center_x, center_y, needle_length, angle)

        shadow_pen = QPen(Qt.black)
        shadow_pen.setWidth(self.needle_width + 4)
        painter.setPen(shadow_pen)
        painter.drawLine(needle_start, needle_end)

        needle_pen = QPen(self.needle_color)
        needle_pen.setWidth(self.needle_width)
        painter.setPen(needle_pen)
        painter.drawLine(needle_start, needle_end)

        hub_radius = size * self.hub_radius_ratio
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(QBrush(Qt.darkGray))
        painter.drawEllipse(QRectF(center_x - hub_radius, center_y - hub_radius, hub_radius * 2, hub_radius * 2))

        if self.unit or True:
            x_ratio, y_ratio, w_ratio, h_ratio = self.value_rect
            value_rect = QRectF(width * x_ratio, height * y_ratio, width * w_ratio, height * h_ratio)

            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(Qt.darkGray))
            painter.drawRoundedRect(value_rect, 12, 12)

            painter.setPen(QPen(Qt.white))
            font = QFont("Arial")
            font.setBold(True)
            font.setPixelSize(max(10, int(size * 24 / 300)))
            painter.setFont(font)

            value = self._value
            if float(value).is_integer():
                value_text = str(int(value))
            else:
                value_text = f"{value:.1f}"
            if self.unit:
                value_text += f" {self.unit}"
            painter.drawText(value_rect, Qt.AlignCenter, value_text)

        painter.end()
