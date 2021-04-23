from parapy.geom import *
from parapy.core import *


class Skid(GeomBase):
    skid_length = Input(2.)
    skid_width = Input(0.3)
    skid_height = Input(0.2)

    @Attribute
    def profiles(self):
        return self.skid_profile

    @Part(in_tree=False)
    def skid_profile(self):
        return Ellipse(quantify=2,
                       major_radius=self.skid_width / 2,
                       minor_radius=self.skid_height / 2,
                       position=rotate90(rotate90(translate(self.position,
                                                            self.position.Vx,
                                                            self.skid_length
                                                            * child.index),
                                                  self.position.Vz),
                                         self.position.Vy))

    @Part
    def skid(self):
        return LoftedSolid(profiles=self.profiles,
                           color='silver')

