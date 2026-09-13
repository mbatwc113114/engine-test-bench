class ResponsiveMapper:

    DESIGN_WIDTH = 1920
    DESIGN_HEIGHT = 1080

    def __init__(self, widget):
        self.widget = widget

    @property
    def scale_x(self):
        return self.widget.width() / self.DESIGN_WIDTH

    @property
    def scale_y(self):
        return self.widget.height() / self.DESIGN_HEIGHT

    def x(self, value):
        return int(value * self.scale_x)

    def y(self, value):
        return int(value * self.scale_y)

    def w(self, value):
        return int(value * self.scale_x)

    def h(self, value):
        return int(value * self.scale_y)

    def rect(self, x, y, width, height, offset_x=0, offset_y=0):

        return (
            self.x(x + offset_x),
            self.y(y + offset_y),
            self.w(width),
            self.h(height)
        )