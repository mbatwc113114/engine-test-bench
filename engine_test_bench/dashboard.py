from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QStackedLayout

from responsive import ResponsiveMapper
from ui.setup_page import SetupPage
from ui.exp_page import ExpPage
from ui.analysis_page import AnalysisPage
from ui.connection_page import ConnectionPage


class Dashboard(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.mapper = ResponsiveMapper(self)
        self.pages = {
            "setup": SetupPage(self),
            "exp": ExpPage(self),
            "analysis": AnalysisPage(self),
            "connection": ConnectionPage(self),
        }
        self.current_page_name = "setup"

        self.setWindowTitle("Engine Test Bench DAQ")
        self.setStyleSheet("""
            QWidget {
                background: #111111;
                color: white;
                font-family: 'Segoe UI';
            }
            QPushButton {
                background: #4a4a4a;
                border: 1px solid #6d6d6d;
                border-radius: 6px;
                color: white;
            }
            QLabel {
                color: white;
            }
        """)

        self.container = QWidget(self)
        self.container_layout = QVBoxLayout(self.container)
        self.container_layout.setContentsMargins(0, 0, 0, 0)
        self.container_layout.setSpacing(0)

        top_bar = QWidget(self)
        top_bar.setObjectName("topBar")
        top_bar.setStyleSheet("background: #2b2b2b; border-bottom: 1px solid #555;")
        top_bar_layout = QHBoxLayout(top_bar)
        top_bar_layout.setContentsMargins(20, 10, 20, 10)

        self.title = QLabel("ENGINE TEST BENCH DAQ")
        self.title.setStyleSheet("font-size: 28px; font-weight: bold; letter-spacing: 1px;")
        self.title.setAlignment(Qt.AlignLeft)

        top_bar_layout.addWidget(self.title)
        top_bar_layout.addStretch()

        self.connection_button = QPushButton("CONNECTION")
        self.connection_button.clicked.connect(lambda: self.show_page("connection"))
        top_bar_layout.addWidget(self.connection_button)

        self.page_nav = QWidget(self)
        page_nav_layout = QHBoxLayout(self.page_nav)
        page_nav_layout.setContentsMargins(20, 12, 20, 12)
        page_nav_layout.setSpacing(12)

        self.setup_button = QPushButton("SETUP")
        self.exp_button = QPushButton("EXP")
        self.analysis_button = QPushButton("ANALYSIS")

        self.setup_button.clicked.connect(lambda: self.show_page("setup"))
        self.exp_button.clicked.connect(lambda: self.show_page("exp"))
        self.analysis_button.clicked.connect(lambda: self.show_page("analysis"))

        page_nav_layout.addWidget(self.setup_button)
        page_nav_layout.addWidget(self.exp_button)
        page_nav_layout.addWidget(self.analysis_button)

        self.stack = QStackedLayout()
        for name, page in self.pages.items():
            self.stack.addWidget(page)

        self.container_layout.addWidget(top_bar)
        self.container_layout.addWidget(self.page_nav)
        self.container_layout.addLayout(self.stack)

        self.setLayout(QVBoxLayout())
        self.layout().addWidget(self.container)

        self.show_page(self.current_page_name)

    def show_page(self, name):
        self.current_page_name = name
        if name == "setup":
            self.stack.setCurrentIndex(0)
        elif name == "exp":
            self.stack.setCurrentIndex(1)
        elif name == "analysis":
            self.stack.setCurrentIndex(2)
        elif name == "connection":
            self.stack.setCurrentIndex(3)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.container.setGeometry(0, 0, self.width(), self.height())


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    dashboard = Dashboard()
    dashboard.resize(1600, 900)
    dashboard.show()
    sys.exit(app.exec_())
