from math import *
from parapy.geom import *
from parapy.core import *
from .lifting_surface import LiftingSurface


class Propeller(GeomBase):
    number_of_blades = Input(2)
    blade_radius = Input(1.2)
    aspect_ratio = Input(10)
    blade_taper_ratio = Input(1)
    thickness_at_root = Input(3.)
    thickness_at_tip = Input(1.)
    ratio_hub_to_blade_radius = Input(0.2)

    leading_edge_sweep = Input(0)
    blade_setting_angle = Input(30)
    blade_outwash = Input(20)

    number_of_blade_sections = Input(50)

    blade_thickness = Input(40)

    @Input
    def blade_profile(self):
        max_thickness = 2400 + self.blade_thickness
        hub = [str(max_thickness)] * int(ceil(self.hub_base_radius /
                                              self.propeller_radius
                                              * self.number_of_blade_sections))
        profile = [str(int(max_thickness - ((index - len(hub))
                                            / (self.number_of_blade_sections
                                               - len(hub)))
                           ** 0.5 * (max_thickness - 2408)))
                   for index in
                   range(len(hub), self.number_of_blade_sections)]
        return hub + profile

    @Input
    def chord_factor(self):
        hub = ([0.2 * self.propeller_radius / self.aspect_ratio]
               * int(ceil(self.hub_base_radius / self.propeller_radius
                          * self.number_of_blade_sections)))
        blade = [self.propeller_radius / self.aspect_ratio * (0.2 + 2.5
                 * (sqrt(index / self.number_of_blade_sections -
                         self.hub_base_radius / self.propeller_radius)
                    - 1.3 * (index / self.number_of_blade_sections -
                             self.hub_base_radius / self.propeller_radius)
                    ** 1.5))
                 for index in range(len(hub), self.number_of_blade_sections)]
        return hub + blade

    @Attribute
    def angle_between_blades(self):
        return 2 * pi / self.number_of_blades

    @Attribute
    def hub_base_radius(self):
        return (self.ratio_hub_to_blade_radius * self.propeller_radius
                if self.ratio_hub_to_blade_radius < 0.4
                else 0.2 * self.propeller_radius)

    @Attribute
    def propeller_radius(self):
        return self.blade_radius if self.blade_radius < 1.8 else 1.8

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
                               center=self.position.point,
                               direction=Vector(0, 0, 1))

    @Part(in_tree=False)
    def propeller(self):
        return LiftingSurface(color='black',
                              is_mirrored=False,
                              number_of_profiles=self.number_of_blade_sections,
                              chord_factor=self.chord_factor,
                              airfoils=self.blade_profile,
                              span=self.propeller_radius,
                              aspect_ratio=self.aspect_ratio,
                              taper_ratio=self.blade_taper_ratio,
                              thickness_factor_root=self.thickness_at_root,
                              thickness_factor_tip=self.thickness_at_tip,
                              sweep=self.leading_edge_sweep,
                              incidence_angle=0,
                              twist=-self.blade_outwash,
                              dihedral=0,
                              position=
                              rotate(translate(self.position,
                                               'z', 0.05),
                                     'y', radians(self.blade_setting_angle)))

    @Part
    def propellers(self):
        return RotatedShape(quantify=self.number_of_blades,
                            color='black',
                            shape_in=self.propeller.surface,
                            rotation_point=self.position.point,
                            vector=Vector(0, 0, 1),
                            angle=self.angle_between_blades * child.index)
