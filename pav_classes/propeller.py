from math import *
from parapy.geom import *
from parapy.core import *
from .lifting_surface import LiftingSurface


class Propeller(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------
    name = Input()

    # Inputs related to the hub
    ratio_hub_to_blade_radius = Input(0.2)

    # Inputs related to the nacelle
    nacelle_included = Input(True)
    nacelle_length = Input(1)

    # Inputs related to the blades
    number_of_blades = Input(6)
    blade_radius = Input(1.)
    aspect_ratio = Input(3.)
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
        hub = ([0.2]
               * int(ceil(self.hub_base_radius / self.propeller_radius
                          * self.number_of_blade_sections)))
        blade = [(0.2 + 2.5 * (sqrt(index / self.number_of_blade_sections
                                    - self.hub_base_radius
                                    / self.propeller_radius)
                               - 1.1 * (index / self.number_of_blade_sections
                                        - self.hub_base_radius
                                        / self.propeller_radius) ** 1.5))
                 for index in range(len(hub), self.number_of_blade_sections)]
        return hub + blade

    @Input
    def hub_length(self):
        return self.hub_base_radius * 1

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

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

    # @Attribute
    # def hub_length(self):
    #     return self.hub_base_radius * 1.5

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

    @Attribute
    def nacelle_locations(self):
        basis_point = self.point_locations[0]
        end_point = translate(self.position,
                              self.position.Vz, - self.nacelle_length)
        return [basis_point, end_point]

    @Attribute
    def nacelle_tangents(self):
        basis_tangent = - self.point_tangents[0]
        end_tangent = self.point_tangents[1]
        return basis_tangent

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    @Part(in_tree=False)
    def hub_profile(self):
        return InterpolatedCurve(points=self.point_locations,
                                 tangents=self.point_tangents)

    @Part
    def hub_cone(self):
        return RevolvedSolid(color='blue',
                             built_from=self.hub_profile,
                             center=self.position.point,
                             direction=self.position.Vz)

    @Part(in_tree=False)
    def nacelle_profile(self):
        return InterpolatedCurve(points=self.nacelle_locations,
                                 initial_tangent=self.nacelle_tangents)

    @Part(in_tree=False)
    def nacelle(self):
        return RevolvedSolid(built_from=self.nacelle_profile,
                             center=self.position.point,
                             direction=self.position.Vz,
                             suppress=not self.nacelle_included)

    @Part(in_tree=False)
    def propeller_reference(self):
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
                                               self.hub_length * 2 / 3),
                                     self.position.Vy,
                                     radians(self.blade_setting_angle)))

    @Part(in_tree=False)
    def propeller(self):
        return SubtractedSolid(shape_in=self.propeller_reference.surface,
                               tool=self.hub_cone)

    @Part
    def propellers(self):
        return RotatedShape(quantify=self.number_of_blades,
                            color='black',
                            shape_in=self.propeller,
                            rotation_point=self.position.point,
                            vector=self.position.Vz,
                            angle=self.angle_between_blades * child.index)
