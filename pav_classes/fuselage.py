from parapy.geom import *
from parapy.core import *


class SquareBox(GeomBase):
    width = Input(3)
    length = Input(2)

    @Attribute
    def height(self):
        return self.width + self.length

    @Part
    def box(self):
        return Box(width=self.width,
                   length=self.length,
                   height=self.height)
