from parapy.geom import *
from parapy.core import *
from .lifting_surface import LiftingSurface


class Skid(GeomBase):
    skid_length = Input(2.)
    skid_width = Input(0.3)
    skid_height = Input(0.2)

    skid_connection_profile = Input('2408')

    chord_skid_connection = Input(0.3)
    thickness_factor_connection = Input(1.)
    span_skid_connection = Input(1.)
    sweep_skid_connection = Input(5.)
    twist_skid_connection = Input(0)
    taper_skid_connection = Input(1.)

    @Attribute
    def profiles(self):
        return self.skid_profile

    @Part(in_tree=False)
    def skid_profile(self):
        return Ellipse(quantify=2,
                       major_radius=self.skid_width,
                       minor_radius=self.skid_height,
                       position=rotate90(rotate90(translate(self.position,
                                                            self.position.Vx,
                                                            2 * child.index),
                                                  self.position.Vz),
                                         self.position.Vy))

    @Part
    def skid(self):
        return LoftedSolid(self.profiles)

    @Part
    def vertical_skid(self):
        return LiftingSurface(name='skid_connections',
                              quantify=2,
                              number_of_profiles=2,
                              airfoils=[self.skid_connection_profile],
                              is_mirrored=False,
                              span=self.span_skid_connection,
                              aspect_ratio=self.span_skid_connection /
                                           self.chord_skid_connection,
                              taper_ratio=self.taper_skid_connection,
                              thickness_factor_root=self.thickness_factor_connection,
                              thickness_factor_tip=self.thickness_factor_connection,
                              sweep=self.sweep_skid_connection,
                              incidence_angle=0,
                              twist=self.twist_skid_connection,
                              dihedral=0,
                              position=rotate90(translate(self.position,
                                                          self.position.Vx,
                                                          self.skid_length -
                                                          self.chord_skid_connection
                                                          * child.index),
                                                self.position.Vx))


if __name__ == '__main__':
    from parapy.gui import display

    obj = Skid(label="skid", mesh_deflection=0.0001)
    display(obj)
