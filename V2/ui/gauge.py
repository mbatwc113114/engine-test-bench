from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QRectF, QPointF
import math


class Gauge(QWidget):

    def __init__(
        self,
        gauge_id,
        min_value=0,
        max_value=100,
        unit="",
        parent=None
    ):
        super().__init__(parent)

        # =====================================================
        # IDENTIFICATION
        # =====================================================

        self.id = gauge_id

        # =====================================================
        # VALUE RANGE
        # =====================================================

        self.min_value = float(min_value)
        self.max_value = float(max_value)

        self._value = self.min_value

        # =====================================================
        # UNIT
        # =====================================================

        self.unit = unit

        # =====================================================
        # DIAL ANGLES
        # =====================================================
        #
        # 0 degree = straight UP
        #
        # MIN = -135°
        # MAX = +135°
        #
        # Total sweep = 270°
        #

        self.start_angle = -120.0
        self.end_angle = 120.0

        # =====================================================
        # DIAL SETTINGS
        # =====================================================

        self.dial_radius_ratio = 0.44

        # Outer circle
        self.outer_circle_width = 5

        # Inner circle
        self.inner_circle_ratio = 0.39
        self.inner_circle_width = 2

        # =====================================================
        # NEEDLE
        # =====================================================

        self.needle_length_ratio = 0.34
        self.needle_width = 5

        # Needle color
        self.needle_color = Qt.red

        # =====================================================
        # CENTER HUB
        # =====================================================

        self.hub_radius_ratio = 0.055

        # =====================================================
        # VALUE DISPLAY
        # =====================================================

        self.show_value = True

        self.value_font = "Arial"
        self.value_font_size = 30

        # Value position
        #
        # x, y, width, height
        #

        self.value_rect = (
            0.25,
            0.71,
            0.50,
            0.20
        )

        # =====================================================
        # TRANSPARENT BACKGROUND
        # =====================================================

        self.setAttribute(
            Qt.WA_TranslucentBackground
        )

        self.setMinimumSize(
            100,
            100
        )

    # =========================================================
    # VALUE PROPERTY
    # =========================================================

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, new_value):
        self.setValue(new_value)

    # =========================================================
    # SET VALUE
    # =========================================================

    def setValue(self, value):

        try:
            value = float(value)

        except (TypeError, ValueError):
            return

        # Clamp value
        value = max(
            self.min_value,
            min(
                self.max_value,
                value
            )
        )

        self._value = value

        self.update()

    # =========================================================
    # GET VALUE
    # =========================================================

    def getValue(self):
        return self._value

    # =========================================================
    # SET RANGE
    # =========================================================

    def setRange(self, minimum, maximum):

        minimum = float(minimum)
        maximum = float(maximum)

        if maximum <= minimum:
            raise ValueError(
                "Maximum must be greater than minimum"
            )

        self.min_value = minimum
        self.max_value = maximum

        self.setValue(self._value)

    # =========================================================
    # SET UNIT
    # =========================================================

    def setUnit(self, unit):

        self.unit = unit

        self.update()

    # =========================================================
    # VALUE TO ANGLE
    # =========================================================

    def valueToAngle(self):

        value_range = (
            self.max_value -
            self.min_value
        )

        if value_range <= 0:
            return self.start_angle

        percentage = (
            self._value -
            self.min_value
        ) / value_range

        percentage = max(
            0.0,
            min(
                1.0,
                percentage
            )
        )

        return (
            self.start_angle
            +
            percentage *
            (
                self.end_angle -
                self.start_angle
            )
        )

    # =========================================================
    # ANGLE TO POINT
    # =========================================================

    def point_from_angle(
        self,
        center_x,
        center_y,
        radius,
        angle
    ):

        # Convert so 0° is at the top
        radians = math.radians(
            angle
        )

        x = (
            center_x
            +
            radius *
            math.sin(radians)
        )

        y = (
            center_y
            -
            radius *
            math.cos(radians)
        )

        return QPointF(
            x,
            y
        )

    # =========================================================
    # PAINT EVENT
    # =========================================================

    def paintEvent(self, event):

        painter = QPainter(self)

        painter.setRenderHint(
            QPainter.Antialiasing,
            True
        )

        # =====================================================
        # SIZE
        # =====================================================

        width = self.width()
        height = self.height()

        size = min(
            width,
            height
        )

        # =====================================================
        # CENTER
        # =====================================================

        center_x = width / 2
        center_y = height / 2

        # =====================================================
        # NO DIAL DRAWING
        # Background already contains the gauge ring and marks.
        # Only dynamic needle/value should be drawn here.
        # =====================================================

        # =====================================================
        # NEEDLE
        # =====================================================

        angle = self.valueToAngle()

        needle_length = (
            size *
            self.needle_length_ratio
        )

        needle_start = self.point_from_angle(
            center_x,
            center_y,
            -size * 0.03,
            angle
        )

        needle_end = self.point_from_angle(
            center_x,
            center_y,
            needle_length,
            angle
        )

        # -----------------------------------------------------
        # Needle shadow
        # -----------------------------------------------------

        shadow_pen = QPen(
            Qt.black
        )

        shadow_pen.setWidth(
            self.needle_width + 4
        )

        painter.setPen(
            shadow_pen
        )

        painter.drawLine(
            needle_start,
            needle_end
        )

        # -----------------------------------------------------
        # Red needle
        # -----------------------------------------------------

        needle_pen = QPen(
            self.needle_color
        )

        needle_pen.setWidth(
            self.needle_width
        )

        painter.setPen(
            needle_pen
        )

        painter.drawLine(
            needle_start,
            needle_end
        )

        # =====================================================
        # CENTER HUB
        # =====================================================

        hub_radius = (
            size *
            self.hub_radius_ratio
        )

        painter.setPen(
            QPen(
                Qt.black,
                2
            )
        )

        painter.setBrush(
            QBrush(
                Qt.darkGray
            )
        )

        painter.drawEllipse(
            QRectF(
                center_x - hub_radius,
                center_y - hub_radius,
                hub_radius * 2,
                hub_radius * 2
            )
        )

        # =====================================================
        # VALUE TEXT
        # =====================================================

        if self.show_value:

            x_ratio = self.value_rect[0]
            y_ratio = self.value_rect[1]
            w_ratio = self.value_rect[2]
            h_ratio = self.value_rect[3]

            value_rect = QRectF(
                width * x_ratio,
                height * y_ratio,
                width * w_ratio,
                height * h_ratio
            )

            # -------------------------------------------------
            # Format value
            # -------------------------------------------------

            if self._value.is_integer():

                value_text = str(
                    int(self._value)
                )

            else:

                value_text = (
                    f"{self._value:.1f}"
                )

            if self.unit:

                value_text += (
                    f" {self.unit}"
                )

            # -------------------------------------------------
            # Font
            # -------------------------------------------------

            font = QFont(
                self.value_font
            )

            font.setBold(
                True
            )

            font.setPixelSize(
                max(
                    10,
                    int(
                        size *
                        self.value_font_size /
                        300
                    )
                )
            )

            painter.setFont(
                font
            )

            painter.setPen(
                QPen(
                    Qt.white
                )
            )

            # -------------------------------------------------
            # Draw value
            # -------------------------------------------------

            painter.drawText(
                value_rect,
                Qt.AlignCenter,
                value_text
            )

        painter.end()