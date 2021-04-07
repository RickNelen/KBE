from math import *
from parapy.geom import *
from parapy.core import *
from .lifting_surface import LiftingSurface


class Propeller(GeomBase):
    name = Input()

    number_of_blades = Input(6)
    blade_radius = Input(1.2)
    aspect_ratio = Input(3.)

    ratio_hub_to_blade_radius = Input(0.2)

    leading_edge_sweep = Input(0)
    blade_setting_angle = Input(40)
    blade_outwash = Input(30)

    number_of_blade_sections = Input(50)
    blade_thickness = Input(60)

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
                                                              * (sqrt(
                    index / self.number_of_blade_sections -
                    self.hub_base_radius / self.propeller_radius)
                                                                 - 1.3 * (
                                                                         index / self.number_of_blade_sections -
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

    @Attribute
    def point_locations(self):
        base_point = translate(self.position,
                               self.position.Vy, self.hub_base_radius)
        tip_point = translate(self.position,
                              self.position.Vz, self.hub_length)
        return [base_point, tip_point]

    @Attribute
    def point_tangents(self):
        base_tangent = self.position.Vz
        tip_tangent = - self.position.Vy
        return [base_tangent, tip_tangent]

    @Part(in_tree=False)
    def hub_profile(self):
        return InterpolatedCurve(points=self.point_locations,
                                 tangents=self.point_tangents)

    @Part
    def hub_cone(self):
        return RevolvedSurface(basis_curve=self.hub_profile,
                               center=Point(self.position.x,
                                            self.position.y,
                                            self.position.z),
                               direction=self.position.Vz)

    @Part(in_tree=False)
    def propeller(self):
        return LiftingSurface(color='black',
                              is_mirrored=False,
                              number_of_profiles=self.number_of_blade_sections,
                              chord_factor=self.chord_factor,
                              airfoils=self.blade_profile,
                              span=self.propeller_radius,
                              aspect_ratio=self.aspect_ratio,
                              taper_ratio=1,
                              thickness_factor_root=1,
                              thickness_factor_tip=1,
                              sweep=self.leading_edge_sweep,
                              incidence_angle=0,
                              twist=-self.blade_outwash,
                              dihedral=0,
                              position=
                              rotate(translate(self.position,
                                               self.position.Vz,
                                               self.hub_length / 3),
                                     self.position.Vy,
                                     radians(self.blade_setting_angle)))

    @Part
    def propellers(self):
        return RotatedShape(quantify=self.number_of_blades,
                            color='black',
                            shape_in=self.propeller.surface,
                            rotation_point=self.position.point,
                            vector=self.position.Vz,
                            angle=self.angle_between_blades * child.index)
