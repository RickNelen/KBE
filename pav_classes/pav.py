import os.path

from math import *
from parapy.geom import *
from parapy.core import *
from parapy.exchange import STEPWriter
import kbeutils.avl as avl

from .fuselage import Fuselage
from .lifting_surface import LiftingSurface
from .airfoil import Airfoil
from .propeller import Propeller
from .skids import Skid

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')
FILENAME = os.path.join(OUTPUT_DIR, 'pav_assembly.stp', '')

g = 9.80665
gamma = 1.4
R = 287


def chord_length(root_chord, tip_chord, span_position):
    return root_chord - (root_chord - tip_chord) * span_position


class PAV(GeomBase):
    name = Input()

    number_of_passengers = Input(4)
    # Range in [km]
    range = Input(500)
    # Maximum allowable span in [m]
    maximum_span = Input(12)
    # Quality level can be 1 for economy or 2 for business
    quality_level = Input(1)
    # Should wheels be added to allow for driving? True or False
    wheels = Input(False)
    # The cruise velocity can be given in [km/hr]
    cruise_velocity = Input(200)
    # The colours that are used for the visualisation
    primary_colour = Input('white')
    secondary_colour = Input('red')

    @Input
    def design_cl(self):
        return 0.5

    @Input
    def cruise_altitude_in_feet(self):
        return 10e3

    @Input
    def number_of_seats_abreast(self):
        return (1 if self.number_of_passengers == 1 else 2 if
        self.number_of_passengers <= 6 else 3)

    @Input
    def length_of_fuselage_nose(self):
        return (1 if self.number_of_passengers <= 4
                else 0.6 * self.number_of_seats_abreast)

    @Input
    def length_of_fuselage_tail(self):
        return (1.5 if self.number_of_passengers <= 4
                else 1 + self.number_of_seats_abreast / 2)

    # -------------------------------------------------------------------------
    # Mostly fuselage related
    # -------------------------------------------------------------------------

    # @Attribute
    # def testing(self):
    #     return str(OUTPUT_DIR)

    @Attribute
    def seat_pitch(self):
        return 1.4 if self.quality_level == 2 else 0.95

    @Attribute
    def seat_width(self):
        return 0.7 if self.quality_level == 2 else 0.5

    @Attribute
    def number_of_rows(self):
        return ceil(self.number_of_passengers / self.number_of_seats_abreast)

    @Attribute
    def cabin_length(self):
        return self.number_of_rows * self.seat_pitch

    @Attribute
    def fuselage_length(self):
        return (self.length_of_fuselage_nose + self.cabin_length +
                self.length_of_fuselage_tail)

    @Attribute
    # Add 20 cm to the cabin width to account for additional things; this
    # assumes that there is no aisle, but each row has its own exits
    def cabin_width(self):
        return self.seat_width * self.number_of_seats_abreast + 0.2

    # Define the height of the cabin
    @Attribute
    def cabin_height(self):
        return self.cabin_width if self.cabin_width >= 1.5 else 1.5

    # -------------------------------------------------------------------------
    # Mostly wing related
    # -------------------------------------------------------------------------
    @Attribute
    def intended_velocity(self):
        # The input velocity is converted from km/h to m/s
        return self.cruise_velocity / 3.6

    @Attribute
    def velocity(self):
        return self.cruise_mach_number * self.cruise_speed_of_sound

    @Attribute
    def max_velocity(self):
        # The maximum velocity is slightly higher than the cruise velocity (
        # which is set at 90% of the maximum velocity)
        return self.velocity / 0.9

    # Describe cruise conditions

    @Attribute
    def cruise_altitude(self):
        # Convert the input altitude in feet to metres
        return self.cruise_altitude_in_feet * 0.3048

    @Attribute
    def cruise_temperature(self):
        return 288.15 - 0.0065 * self.cruise_altitude

    @Attribute
    def cruise_density(self):
        return (1.225 * (self.cruise_temperature / 288.15)
                ** (-1 - g / (R * -0.0065)))

    @Attribute
    def cruise_speed_of_sound(self):
        return sqrt(gamma * R * self.cruise_temperature)

    @Attribute
    def cruise_mach_number(self):
        mach = self.intended_velocity / self.cruise_speed_of_sound
        return mach if mach < 0.6 else 0.6

    @Attribute
    def maximum_take_off_weight(self):
        # MTOW in Newtons
        return (2000 + self.number_of_passengers * 70) * g

    # -------------------------------------------------
    # TO DO: implement correct formula!!!!
    # -------------------------------------------------

    @Attribute
    def wing_area(self):
        return self.maximum_take_off_weight / (0.5 * self.cruise_density *
                                               self.velocity ** 2 *
                                               self.design_cl)

    @Attribute
    def intended_wing_aspect_ratio(self):
        return 10

    @Attribute
    def wing_aspect_ratio(self):
        return self.wing_span ** 2 / self.wing_area

    @Attribute
    def wing_span(self):
        span = sqrt(self.intended_wing_aspect_ratio * self.wing_area)
        return span if span < self.maximum_span else self.maximum_span

    @Attribute
    def wing_sweep(self):
        return 10 if self.cruise_mach_number < 0.4 else 10 + (
                self.cruise_mach_number - 0.4) * 50

    @Attribute
    def wing_dihedral(self):
        return 0 if self.wing_location.z > 0 else 2

    @Attribute
    def wing_location(self):
        length_ratio = 0.3
        height_ratio = 0.8
        return self.position.translate('x', length_ratio *
                                       self.fuselage_length,
                                       'z', (height_ratio - 0.5)
                                       * self.cabin_height)

    @Attribute
    def number_of_propellers(self):
        return ceil(self.wing_area / 2)

    @Attribute
    def propeller_radii(self):
        return ([1 - 0.1 * self.number_of_propellers]
                * self.number_of_propellers)

    @Attribute
    def propeller_locations(self):
        semi_span = self.wing_span / 2
        sweep = radians(self.wing_sweep)
        dihedral = radians(self.wing_dihedral)
        # If there is an odd number of propellers, the first propeller is
        # located at the nose of the plane
        first = translate(self.wing_location,
                          self.position.Vx,
                          - self.wing_location.x,
                          self.position.Vz,
                          - self.wing_location.z
                          + self.fuselage.nose_height * self.cabin_height)

        # Determine the number of propellers on each side
        one_side = (int(self.number_of_propellers / 2)
                    if self.number_of_propellers % 2 != 0
                    else int((self.number_of_propellers - 1) / 2))

        # Position each propeller in y-direction on one wing; make sure they
        # are placed such that the most inboard propeller tip still is 0.5
        # propeller radius away from the fuselage
        y_shift = [(1 - (semi_span - self.cabin_width / 2
                         - 1.5 * self.propeller_radii[-1]) / semi_span
                    * index / one_side)
                   * self.wing_span / 2
                   for index in range(one_side)]

        # Place the propellers just ahead of the leading edge of the right wing
        right_wing = [translate(self.wing_location,
                                self.wing_location.Vx,
                                y_shift[index] * tan(sweep)
                                - 0.3 * chord_length(
                                    self.main_wing.root_chord,
                                    self.main_wing.tip_chord,
                                    y_shift[index] / semi_span)
                                - self.propeller_radii[index] * tan(sweep),
                                self.wing_location.Vy,
                                y_shift[index],
                                self.wing_location.Vz,
                                y_shift[index] * tan(dihedral))
                      for index in range(one_side)]

        # Place the propellers just ahead of the leading edge of the left wing
        left_wing = [translate(right_wing[index],
                               self.wing_location.Vy,
                               - 2 * y_shift[index])
                     for index in range(one_side)]
        return [first] + right_wing + left_wing

    @Attribute
    def avl_surfaces(self):
        return self.find_children(lambda o: isinstance(o, avl.Surface))

    @Part
    def main_wing(self):
        return LiftingSurface(name='main_wing',
                              number_of_profiles=4,
                              airfoils=['34018', '34015', '43010', '43008'],
                              is_mirrored=True,
                              span=self.wing_span,
                              aspect_ratio=self.wing_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep,
                              incidence_angle=0,
                              twist=-3,
                              dihedral=self.wing_dihedral,
                              position=self.wing_location,
                              color=self.secondary_colour)

    @Part
    def horizontal_tail(self):
        return LiftingSurface(name='horizontal_tail',
                              number_of_profiles=2,
                              airfoils=['2218', '2212'],
                              is_mirrored=True,
                              span=self.wing_span / 3,
                              aspect_ratio=self.wing_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep + 5,
                              incidence_angle=0,
                              twist=0,
                              dihedral=3,
                              position=self.position.translate(
                                  'x', self.fuselage_length * 0.8,
                                  'z', 0.3 * self.cabin_height),
                              color=self.secondary_colour)

    @Part
    def vertical_tail(self):
        return LiftingSurface(name='vertical_tail',
                              number_of_profiles=2,
                              airfoils=['0012', '0012'],
                              is_mirrored=False,
                              span=self.wing_span / 8,
                              aspect_ratio=self.wing_aspect_ratio / 4,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep + 20,
                              incidence_angle=0,
                              twist=0,
                              dihedral=0,
                              position=rotate90(
                                  self.horizontal_tail.position, 'x'),
                              color=self.primary_colour)

    @Part(in_tree=True)
    def fuselage(self):
        return Fuselage(name='fuselage',
                        number_of_positions=50,
                        nose_fineness=(self.length_of_fuselage_nose
                                       / self.cabin_width),
                        tail_fineness=(self.length_of_fuselage_tail
                                       / self.cabin_width),
                        width=self.cabin_width,
                        cabin_height=self.cabin_height,
                        cabin_length=self.cabin_length,
                        nose_radius_height=0.1,
                        tail_radius_height=0.05,
                        nose_height=-0.2,
                        tail_height=0.3,
                        color=self.primary_colour)

    @Part(in_tree=True)
    def cruise_propellers(self):
        return Propeller(name='cruise_propellers',
                         quantify=len(self.propeller_locations),
                         number_of_blades=6,
                         blade_radius=self.propeller_radii[child.index],
                         nacelle_length=(0.55 * chord_length(
                             self.main_wing.root_chord,
                             self.main_wing.tip_chord,
                             abs(self.propeller_locations[
                                     child.index].y / (self.wing_span / 2)))
                                         + self.propeller_radii[child.index]
                                         * tan(radians(self.wing_sweep))),
                         aspect_ratio=3,
                         ratio_hub_to_blade_radius=0.15,
                         leading_edge_sweep=0,
                         blade_setting_angle=40,
                         blade_outwash=30,
                         number_of_blade_sections=50,
                         blade_thickness=60,
                         position=rotate90(
                             self.propeller_locations[child.index],
                             - self.position.Vy))

    # -------------------------------------------------------------------------
    # SKIDS - REMOVE LATER
    # -------------------------------------------------------------------------

    @Part
    def skid(self):
        return Skid()

    # -------------------------------------------------------------------------
    # AVL part for analysis
    # -------------------------------------------------------------------------

    @Part
    def avl_configuration(self):
        return avl.Configuration(name='pav',
                                 reference_area=self.wing_area,
                                 reference_span=self.wing_span,
                                 reference_chord=1.2,
                                 reference_point=self.main_wing.position.point,
                                 surfaces=self.avl_surfaces,
                                 mach=self.cruise_mach_number)

    # ADJUST THE REFERENCE CHORD AND REFERENCE POINT!

    # -------------------------------------------------------------------------
    # STEP parts used for export
    # -------------------------------------------------------------------------

    @Part
    def step_parts(self):
        return STEPWriter(filename=FILENAME,
                          trees=[self])
