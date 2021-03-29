from math import *
from parapy.geom import *
from parapy.core import *
from .lifting_surface import LiftingSurface


class Propeller(GeomBase):
    number_of_blades = Input(2)
    blade_radius = Input(1.2)
    blade_mean_chord = Input(0.2)
    blade_taper_ratio = Input(0.3)
    thickness_at_root = Input(3.)
    thickness_at_tip = Input(1.)
    ratio_hub_to_blade_radius = Input(0.2)

    leading_edge_sweep = Input(10)
    blade_setting_angle = Input(30)
    blade_outwash = Input(20)

    number_of_blade_sections = Input(8)
    blade_profile = Input(['2410'])

    @Input
    def chord_factor(self):
        return [
            3. / 4 + sqrt(index / self.number_of_blade_sections + 9 / 16) - (
                    index / self.number_of_blade_sections) ** 2 for index in
            range(self.number_of_blade_sections)]

    @Attribute
    def angle_between_blades(self):
        return 2 * pi / self.number_of_blades

    @Attribute
    def hub_base_radius(self):
        return (self.ratio_hub_to_blade_radius * self.blade_radius
                if self.ratio_hub_to_blade_radius < 0.4
                else 0.2 * self.blade_radius)

    @Attribute
    def hub_length(self):
        return self.hub_base_radius * 1.5

    @Part(in_tree=False)
    def hub_profile(self):
        return InterpolatedCurve(points=[Point(x=0, y=self.hub_base_radius,
                                               z=0), Point(x=0, y=0,
                                                           z=self.hub_length)],
                                 tangents=[Vector(0, 0, 1), Vector(0, -1, 0)])

    @Part
    def hub_cone(self):
        return RevolvedSurface(basis_curve=self.hub_profile,
                               center=Point(0, 0, 0),
                               direction=Vector(0, 0, 1))

    @Part
    def propellers(self):
        return LiftingSurface(quantify=self.number_of_blades,
                              color='black',
                              is_mirrored=False,
                              number_of_profiles=self.number_of_blade_sections,
                              chord_factor=self.chord_factor,
                              airfoils=self.blade_profile,
                              span=self.blade_radius,
                              aspect_ratio=(self.blade_radius
                                            / self.blade_mean_chord),
                              taper_ratio=self.blade_taper_ratio,
                              thickness_factor_root=self.thickness_at_root,
                              thickness_factor_tip=self.thickness_at_tip,
                              sweep=self.leading_edge_sweep,
                              incidence_angle=self.blade_setting_angle,
                              twist=self.blade_outwash,
                              dihedral=0,
                              position=translate(rotate(self.position,
                                                        'z',
                                                        self.angle_between_blades *
                                                        child.index), 'z',
                                                 self.blade_mean_chord))
