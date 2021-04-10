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
from .wheels import Wheels, Rods
from .avl_configurator import AvlAnalysis

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')
FILENAME = os.path.join(OUTPUT_DIR, 'pav_assembly.stp', '')

g = 9.80665
gamma = 1.4
R = 287

lbs_to_kg = 0.45359237
ft_to_m = 0.3048

C_T = 0.0050  # between 0.0050 and 0.0060
sigma_rotor = 0.070  # fixed
Mtip = 0.6  # fixed, higher Mach numbers for the rotor tips lead to stall
DL_max = 1500  # fixed


def chord_length(root_chord, tip_chord, span_position):
    return root_chord - (root_chord - tip_chord) * span_position


cases = [('fixed_aoa', {'alpha': 3}),
         ('fixed_cl', {'alpha': avl.Parameter(name='alpha',
                                              value='0.3',
                                              setting='CL')})]


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
    wheels_included = Input(False)
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
    # Centre of gravity related
    # -------------------------------------------------------------------------

    @Attribute
    def pav_components(self):
        return self.find_children(lambda o: isinstance(o, (LoftedSolid,
                                                           RevolvedSolid,
                                                           RotatedShape)))

    @Attribute
    def center_of_gravity_of_components(self):
        return [component.cog for component in self.pav_components]

    @Attribute
    def mass_of_components(self):
        horizontal_tail_t_over_c = float(
            self.horizontal_tail.airfoils[0]) / 100
        vertical_tail_t_over_c = float(
            self.vertical_tail.airfoils[0]) / 100

        # These tail volume coefficients assume general aviation - twin
        # engine aircraft; CHECK ONCE MORE IF THIS IS WORKING WELL! (see
        # slide 7)
        horizontal_tail_volume_coefficient = 0.8
        vertical_tail_volume_coefficient = 0.07

        # Replace these things by measured items and proper values later!
        ultimate_load_factor = 3.
        root_t_over_c = 0.12

        control_surface_area = 0.1 * self.wing_area
        k_door = 1.06
        k_lg = 1.0
        k_ws = 0.95
        l_over_d = 20
        # --------
        mass_wing = (0.0051 * (self.maximum_take_off_weight / lbs_to_kg
                               * ultimate_load_factor) ** 0.557
                     * (self.wing_area / (ft_to_m ** 2)) ** 0.649
                     * sqrt(self.wing_aspect_ratio) * root_t_over_c ** -0.4
                     * (1 + self.main_wing.taper_ratio) ** 0.1
                     * (cos(radians(self.wing_sweep))) ** -1
                     * control_surface_area ** 0.1)
        mass_fuselage = (0.3280 * k_door * k_lg *
                         (self.maximum_take_off_weight / lbs_to_kg
                          * ultimate_load_factor) ** 0.5
                         * (self.fuselage_length / ft_to_m) ** 0.25

                         # THIS LINE TRIES TO GET THE AREA OF THE FUSELAGE BUT
                         # DOES NOT WORK YET
                         * (self.pav_components[10].area / (ft_to_m ** 2))
                         ** 0.302
                         * (1 + k_ws) ** 0.04 * l_over_d ** 0.1)
        mass_horizontal_tail = (0.016 *
                                (self.maximum_take_off_weight / lbs_to_kg)
                                ** 0.414
                                * (0.5 * self.cruise_density
                                   * self.velocity ** 2)
                                ** 0.168
                                * self.horizontal_tail_area ** 0.896
                                * (100 * horizontal_tail_t_over_c /
                                   cos(radians(
                                       self.horizontal_tail_sweep))) ** -0.12)
        mass_vertical_tail = (0.073 *
                              (1 + 0.2 * horizontal_tail_volume_coefficient
                               / vertical_tail_volume_coefficient)
                              * (self.maximum_take_off_weight / lbs_to_kg
                                 * ultimate_load_factor) ** 0.376
                              * (0.5 * self.cruise_density * self.velocity
                                 ** 2) ** 0.122
                              * self.vertical_tail_area ** 0.873
                              * (100 * vertical_tail_t_over_c /
                                 cos(radians(self.vertical_tail_sweep)))
                              ** - 0.49)
        mass_landing_gear = 20 * len(self.wheel_locations) * 2

        return [mass_wing, mass_fuselage,
                mass_horizontal_tail, mass_vertical_tail, mass_landing_gear]

    # -------------------------------------------------------------------------
    # Battery related
    # -------------------------------------------------------------------------

    @Attribute
    def drag(self):
        analysis = AvlAnalysis(case_settings=cases)
        return analysis.lift_over_drag

    @Attribute
    def battery_energy_density(self):
        # 200 Wh per kg converted to Joule per kg
        return 200 * 3600

    @Attribute
    def battery_discharge_time(self):
        # The factor 3/2 is included to compensate for slower flight phases
        # such as take-off and approach
        return self.range * 1000 / self.velocity * 3 / 2

    # -------------------------------------------------------------------------
    # Horizontal tail related
    # -------------------------------------------------------------------------

    # ADJUST THIS THING !!!!!!!!!!

    @Attribute
    def horizontal_tail_area(self):
        return 10

    @Attribute
    def vertical_tail_area(self):
        return 8

    # ADJUST THIS THING !!!!!!!!!!
    @Attribute
    def horizontal_tail_sweep(self):
        return 30

    @Attribute
    def vertical_tail_sweep(self):
        return 20

    # -------------------------------------------------------------------------
    # Propeller related
    # -------------------------------------------------------------------------

    N_blades = Input(4)  # between 2 and 5
    chord_rotor = Input(0.08)
    R_rotor = Input(.4)
    ROC_vertical = Input(10)  # between 5 and 15 m/s is most common
    n_rotors = Input(4)  # should be even, >2
    C_D_rotor = Input(0.1)  # obtain from some analysis somewhere

    # still needs to be added: constant of R/chord, link with length
    # skids, find C_D, then get a formula between R and n -> only variables
    # are R/chord, n, N

    @Attribute
    def Disk_Loading(self):
        return (self.T_max_vertical / (pi * self.R_rotor ** 2)
                if self.T_max_vertical / (pi * self.R_rotor ** 2) <= DL_max
                else "DL is too high")

    @Attribute
    def T_max_vertical(self):
        return self.P_climb / self.ROC_vertical

    @Attribute
    def P_climb(self):
        return (self.P_hover + self.P_ROC + self.P_profile +
                self.P_D_liftingsurface)

    @Attribute
    def P_hover(self):
        return self.T_hover * self.V_iclimb

    # CT IS UNKNOWN! WHAT IS ITS VALUE?
    @Attribute
    def T_hover(self):
        return (1. / 6. * self.N_blades * 6.6 * CT / sigma_rotor
                * self.cruise_density * self.chord_rotor
                * (0.97 * Mtip * (
                    sqrt(R * gamma * self.cruise_temperature))) ** 2
                * 0.97 * self.R_rotor * self.n_rotors)

    @Attribute
    def V_iclimb(self):
        return (self.ROC_vertical / 2
                + sqrt((self.ROC_vertical / 2) ** 2 * self.T_hover
                       / (2 * self.cruise_density * pi * self.R_rotor)))

    @Attribute
    def P_ROC(self):
        return self.MTOW * self.ROC_vertical / 2

    @Attribute
    def P_profile(self):
        return (self.C_d_rotor * 1. / 8. * self.cruise_density *
                self.chord_rotor * self.N_blades
                * (Mtip * (sqrt(R * gamma * self.cruise_temperature))) ** 3
                * self.R_rotor * self.n_rotors)

    @Attribute
    def P_D_liftingsurface(self):
        return self.D_liftingsurfaces * self.ROC_vertical

    @Attribute
    def D_liftingsurfaces(self):
        return (1. / 2. * self.cruise_density * self.ROC_vertical ** 2
                * self.C_D_flatplate
                * (self.wing_area + self.horizontal_tail_area))

    # -------------------------------------------------------------------------
    # Mostly fuselage related
    # -------------------------------------------------------------------------

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
        return 5 if self.wing_location.z > 0 else 3

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

    # ADJUST LATER:
    @Attribute
    def length_of_skids(self):
        return self.fuselage_length * 0.8

    @Attribute
    def vertical_skid_profile(self):
        return '0018'

    @Attribute
    def vertical_skid_chord(self):
        return 0.75

    @Attribute
    def skid_height(self):
        return min(0.1, 0.9 * self.skid_width)

    @Attribute
    def skid_width(self):
        return (1.05 * self.vertical_skid_chord *
                (float(self.vertical_skid_profile) / 100))

        # ADJUST LATER

    @Attribute
    def skid_locations(self):
        return [translate(self.position,
                          self.position.Vx,
                          self.fuselage_length * 0.1,
                          self.position.Vy,
                          - 0.75 * self.cabin_width
                          + 1.5 * self.cabin_width * index,
                          self.position.Vz,
                          (- (0.5 + 0.2) * self.cabin_height)
                          if self.wheels_included is False
                          else - (0.5 + 0.2) * self.cabin_height
                               + self.wheel_radius + self.vertical_rod_length)
                for index in range(2)]

    # ADJUST THIS LATER
    @Attribute
    def wheel_locations(self):
        expected_number_of_wheels = self.wing_aspect_ratio  # Adjust this!!!!
        number_of_wheels = (expected_number_of_wheels
                            if expected_number_of_wheels >= 4 else 4)
        wheels_per_side = (ceil(number_of_wheels / 2)
                           if number_of_wheels * self.wheel_radius
                              < 0.8 * self.length_of_skids
                           else ceil(0.8 * self.length_of_skids /
                                     (2 * self.wheel_radius)))
        left_locations = [translate(self.skid_locations[0],
                                    self.position.Vx,
                                    (index + 0.5) / wheels_per_side
                                    * self.length_of_skids,
                                    - self.position.Vy,
                                    self.horizontal_rod_length
                                    + self.wheel_width - self.rod_radius / 2,
                                    self.position.Vz,
                                    - self.vertical_rod_length)
                          for index in range(wheels_per_side)]
        # Only the locations for the wheels on the left skid are returned,
        # as the wheels on the right skids are simply mirrored
        return left_locations

    @Attribute
    def wheel_radius(self):
        return 0.2

    @Attribute
    def wheel_width(self):
        return 0.15

    @Attribute
    def rod_radius(self):
        return min(0.03, self.skid_width * 0.4)

    @Attribute
    def vertical_rod_length(self):
        return self.wheel_radius * 1.2

    @Attribute
    def horizontal_rod_length(self):
        return self.skid_width * 0.7 + self.rod_radius

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
                              color='silver')

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
                                  self.position.Vx, self.fuselage_length * 0.8,
                                  self.position.Vz, 0.3 * self.cabin_height),
                              color='silver')

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
                         nacelle_included=
                         False if child.index == 0
                                  and len(self.propeller_locations) % 2 == 1
                         else True,
                         aspect_ratio=3,
                         ratio_hub_to_blade_radius=0.15,
                         leading_edge_sweep=0,
                         blade_setting_angle=40,
                         blade_outwash=30,
                         number_of_blade_sections=50,
                         blade_thickness=60,
                         position=rotate90(
                             self.propeller_locations[child.index],
                             - self.position.Vy),
                         color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # SKIDS
    # -------------------------------------------------------------------------

    @Part
    def skids(self):
        return Skid(quantify=2,
                    color=self.secondary_colour,
                    skid_length=self.length_of_skids,
                    skid_width=self.skid_width,
                    skid_height=self.skid_height,
                    skid_connection_profile=self.vertical_skid_profile,
                    chord_skid_connection=self.vertical_skid_chord,
                    # Connect the skid to the horizontal tail
                    span_skid_connection=
                    self.horizontal_tail.position.z
                    + abs(self.skid_locations[child.index].y)
                    * tan(radians(self.horizontal_tail.dihedral))
                    - self.skid_locations[child.index].z,
                    position=self.skid_locations[child.index])

    # -------------------------------------------------------------------------
    # WHEELS
    # -------------------------------------------------------------------------

    @Part
    def left_wheels(self):
        return Wheels(quantify=len(self.wheel_locations),
                      wheel_length=self.wheel_width,
                      wheel_radius=self.wheel_radius,
                      position=self.wheel_locations[child.index],
                      color='black',
                      suppress=not self.wheels_included)

    @Part
    def right_wheels(self):
        return MirroredShape(quantify=len(self.wheel_locations),
                             shape_in=self.left_wheels[child.index].wheel,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='black',
                             suppress=not self.wheels_included)

    @Part
    def left_wheel_rods(self):
        return Rods(quantify=len(self.wheel_locations),
                    wheel_length=self.wheel_width,
                    rod_horizontal_length=self.horizontal_rod_length,
                    rod_vertical_length=self.vertical_rod_length,
                    position=self.wheel_locations[child.index],
                    color='silver',
                    suppress=not self.wheels_included)

    @Part
    def right_wheel_rods(self):
        return MirroredShape(quantify=len(self.wheel_locations),
                             shape_in=self.left_wheel_rods[child.index].rods,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='silver',
                             suppress=not self.wheels_included)

    # -------------------------------------------------------------------------
    # AVL part for analysis
    # -------------------------------------------------------------------------

    @Part
    def avl_configuration(self):
        return avl.Configuration(name='pav',
                                 reference_area=self.wing_area,
                                 reference_span=self.wing_span,
                                 reference_chord=
                                 self.main_wing.mean_aerodynamic_chord,
                                 reference_point=
                                 self.main_wing.profile_locations[0],
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
