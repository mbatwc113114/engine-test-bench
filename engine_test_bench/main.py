import sys

from PyQt5.QtWidgets import QApplication

from dashboard import Dashboard


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Engine Test Bench DAQ")

    window = Dashboard()
    window.resize(1600, 900)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
