from PyQt5.QtWidgets import QWidget, QLabel, QPushButton, QComboBox
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

from ui.gauge import Gauge
from ui.responsive import ResponsiveMapper

class Dashboard(QWidget):

    def __init__(self, parent=None):

        super().__init__(parent)

        self.mapper = ResponsiveMapper(self)

        # ==========================================
        # BACKGROUND
        # ==========================================

        self.background = QLabel(self)

        self.background_pixmap = QPixmap(
            "assets/dashboard.png"
        )

        # ==========================================
        # CREATE CONTROLS
        # ==========================================

        self.create_controls()

        # ==========================================
        # INITIAL LAYOUT
        # ==========================================

        self.update_layout()
        self.update_gauges()

    # =================================================
    # CREATE CONTROLS
    # =================================================

    def create_controls(self):

        self.transparent_button_style = """
QPushButton {
    background: transparent;
    border: none;
    color: transparent;
}

QPushButton:hover {
    background: rgba(255,255,255,20);
}

QPushButton:pressed {
    background: rgba(255,255,255,20);
}
"""

        # ---------------------------------------------
        # Connection dropdown
        # ---------------------------------------------

        self.connection_dropdown = QComboBox(self)

        self.connection_dropdown.addItems([
            "ESP32 UDP connection",
            "ESP32 #1",
            "ESP32 #2"
        ])

        # ---------------------------------------------
        # Connect button
        # ---------------------------------------------

        self.connect_button = QPushButton(
            "CONNECT",
            self
        )
        self.connect_button.setStyleSheet(self.transparent_button_style)

        self.connect_button.clicked.connect(
            self.connect_esp32
        )

        # ---------------------------------------------
        # Live view
        # ---------------------------------------------

        self.live_view = QPushButton(
            "LIVE VIEW",
            self
        )
        self.live_view.setStyleSheet(self.transparent_button_style)

        # ---------------------------------------------
        # Tabs
        # ---------------------------------------------

        self.setup_button = QPushButton(
            "SETUP",
            self
        )
        self.setup_button.setStyleSheet(self.transparent_button_style)

        self.exp_button = QPushButton(
            "EXP",
            self
        )
        self.exp_button.setStyleSheet(self.transparent_button_style)

        self.analysis_button = QPushButton(
            "ANALYSIS",
            self
        )
        self.analysis_button.setStyleSheet(self.transparent_button_style)

        # ---------------------------------------------
        # Gauges
        # ---------------------------------------------

        self.rpm_gauge = Gauge(
            gauge_id="rpm",
            min_value=0,
            max_value=12000,
            unit="RPM",
            parent=self
        )

        self.thrust_gauge = Gauge(
            gauge_id="thrust",
            min_value=0,
            max_value=10,
            unit="kg",
            parent=self
        )

        self.fuel_gauge = Gauge(
            gauge_id="fuel",
            min_value=0,
            max_value=100,
            unit="%",
            parent=self
        )

        self.cht_gauge = Gauge(
            gauge_id="cht",
            min_value=0,
            max_value=250,
            unit="°C",
            parent=self
        )

        self.egt_gauge = Gauge(
            gauge_id="egt",
            min_value=0,
            max_value=1000,
            unit="°C",
            parent=self
        )

        # ---------------------------------------------
        # Bottom buttons
        # ---------------------------------------------

        self.button_1 = QPushButton("BUTTON 1", self)
        self.button_2 = QPushButton("BUTTON 2", self)
        self.button_3 = QPushButton("BUTTON 3", self)
        self.button_4 = QPushButton("BUTTON 4", self)
        self.button_5 = QPushButton("BUTTON 5", self)

        for button in [
            self.button_1,
            self.button_2,
            self.button_3,
            self.button_4,
            self.button_5,
        ]:
            button.setStyleSheet(self.transparent_button_style)

    # =================================================
    # RESPONSIVE LAYOUT
    # =================================================

    def update_layout(self):

        m = self.mapper
        offset_x = 10
        offset_y = 3

        # ---------------------------------------------
        # Connection dropdown
        # ---------------------------------------------

        self.connection_dropdown.setGeometry(
            *m.rect(
                950,
                18,
                446,
                57,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # Connect
        # ---------------------------------------------

        self.connect_button.setGeometry(
            *m.rect(
                1419,
                18,
                350,
                57,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # Live View
        # ---------------------------------------------

        self.live_view.setGeometry(
            *m.rect(
                55,
                141,
                334,
                53,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # SETUP
        # ---------------------------------------------

        self.setup_button.setGeometry(
            *m.rect(
                1095,
                159,
                158,
                67,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # EXP
        # ---------------------------------------------

        self.exp_button.setGeometry(
            *m.rect(
                1260,
                159,
                159,
                67,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # ANALYSIS
        # ---------------------------------------------

        self.analysis_button.setGeometry(
            *m.rect(
                1427,
                159,
                160,
                67,
                offset_x=offset_x,
                offset_y=offset_y
            )
        )

        # ---------------------------------------------
        # Bottom buttons
        # ---------------------------------------------

        self.button_1.setGeometry(
            *m.rect(84, 950, 158, 69, offset_x=offset_x, offset_y=offset_y)
        )

        self.button_2.setGeometry(
            *m.rect(277, 950, 158, 69, offset_x=offset_x, offset_y=offset_y)
        )

        self.button_3.setGeometry(
            *m.rect(464, 950, 158, 69, offset_x=offset_x, offset_y=offset_y)
        )

        self.button_4.setGeometry(
            *m.rect(653, 950, 158, 69, offset_x=offset_x, offset_y=offset_y)
        )

        self.button_5.setGeometry(
            *m.rect(840, 950, 158, 69, offset_x=offset_x, offset_y=offset_y)
        )

        self.update_gauges()

    def update_gauges(self):

        m = self.mapper

        self.rpm_gauge.setGeometry(
            *m.rect(80, 245, 300, 300, offset_x=0, offset_y=-10)
        )

        self.thrust_gauge.setGeometry(
            *m.rect(420, 245, 300, 300, offset_x=-5, offset_y=-8)
        )

        self.fuel_gauge.setGeometry(
            *m.rect(760, 245, 300, 300, offset_x=-10, offset_y=-10)
        )

        self.cht_gauge.setGeometry(
            *m.rect(245, 565, 300, 300, offset_x=25, offset_y=-8)
        )

        self.egt_gauge.setGeometry(
            *m.rect(580, 565, 300, 300, offset_x=25, offset_y=-8)
        )

    def update_background(self):

        self.background.setGeometry(
            0,
            0,
            self.width(),
            self.height()
        )

        scaled = self.background_pixmap.scaled(
            self.width(),
            self.height(),
            Qt.IgnoreAspectRatio,
            Qt.SmoothTransformation
        )

        self.background.setPixmap(scaled)

    # =================================================
    # RESIZE EVENT
    # =================================================

    def resizeEvent(self, event):

        self.update_background()
        self.update_layout()
        self.update_gauges()

        super().resizeEvent(event)

    # =================================================
    # BUTTON FUNCTIONS
    # =================================================

    def connect_esp32(self):

        print("CONNECT pressed")