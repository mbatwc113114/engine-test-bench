import sys

from PyQt5.QtWidgets import QApplication, QMainWindow
from ui.dashboard import Dashboard


class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Engine Test Bench")

        # Starting size
        self.resize(1900, 1080)

        # Minimum size
        self.setMinimumSize(800, 500)

        # Dashboard
        self.dashboard = Dashboard(self)

        self.setCentralWidget(self.dashboard)


if __name__ == "__main__":

    app = QApplication(sys.argv)

    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())