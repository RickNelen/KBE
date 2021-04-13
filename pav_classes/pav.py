import os.path

from math import *
from parapy.geom import *
from parapy.core import *
from parapy.core.validate import *
from parapy.exchange import STEPWriter
import kbeutils.avl as avl
import warnings

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

c_t = 0.0050  # between 0.0050 and 0.0060
sigma_rotor = 0.070  # fixed
mach_number_tip = 0.6  # fixed, higher Mach numbers for the rotor tips lead to stall
dl_max = 1500  # fixed
r_over_chord_rotor: int = 15 # between 15 and 20
figure_of_merit = 0.8  # between 0.6 and 0.8
twist_rotor = -10  # degrees, tip angle is 10 degrees lower than at root, which is good for hover
k_factor_rotor_drag = 1.15 # between 1.1 and 1.2

design_lift_coefficient = 0.5


def generate_warning(warning_header, message):
    from tkinter import Tk, mainloop, X, messagebox

    window = Tk()
    window.withdraw()
    messagebox.showwarning(warning_header, message)


def chord_length(root_chord, tip_chord, span_position):
    return root_chord - (root_chord - tip_chord) * span_position


cases = [('fixed_cl',
          {'alpha': avl.Parameter(name='alpha',
                                  value=str(design_lift_coefficient),
                                  setting='CL')})]


class PAV(GeomBase):
    name = Input()

    number_of_passengers = Input(4, validator=And(Positive, LessThan(20)))
    # Range in [km]
    range = Input(500)
    # Maximum allowable span in [m]
    maximum_span = Input(12)
    # Quality level can be 1 for economy or 2 for business
    quality_level = Input(1, validator=OneOf([1, 2]))
    # Should wheels be added to allow for driving? True or False
    wheels_included = Input(True)
    # The cruise velocity can be given in [km/hr]
    set_cruise_velocity = Input(400)
    # The colours that are used for the visualisation
    primary_colour = Input('white')
    secondary_colour = Input('red')

    @Input
    def design_cl(self):
        return design_lift_coefficient

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

    # Checks

    @Attribute
    def cruise_velocity(self):
        if self.set_cruise_velocity > 400:
            message = 'The cruise velocity is set too high. The cruise ' \
                      'velocity will be set to 400 km/h.'
            generate_warning('Warning: value changed', message)
            return 300
        else:
            return self.set_cruise_velocity

    # -------------------------------------------------------------------------
    # Centre of gravity related
    # -------------------------------------------------------------------------

    @Attribute
    def pav_components(self):
        names = ['main_wing', 'horizontal_tail', 'vertical_tail',
                 'fuselage', 'propeller', 'skid', 'wheel']

        right_wing = self.main_wing.surface
        # left_wing = self.main_wing.mirrored
        right_horizontal_tail = self.horizontal_tail.surface
        # left_horizontal_tail = self.horizontal_tail.mirrored
        right_vertical_tail = self.vertical_tail[1].surface
        # left_vertical_tail = self.skids[0].vertical_skid.surface
        fuselage = self.fuselage.fuselage_cabin
        propeller = [self.cruise_propellers[index].hub_cone
                     for index in range(len(self.propeller_locations))]
        skid = [self.skids[index].skid
                for index in range(len(self.skid_locations))]
        left_wheels = [self.left_wheels[index].wheel
                       for index in range(len(self.wheel_locations))]
        right_wheels = [self.right_wheels[index]
                        for index in range(len(self.wheel_locations))]
        wheels = left_wheels  # + right_wheels

        # return ([right_wing] + [right_horizontal_tail]
        #         + [right_vertical_tail]
        #         + [left_vertical_tail] + [fuselage] + propeller
        #         + skid + wheels)

        return {'main_wing': right_wing,
                'horizontal_tail': right_horizontal_tail,
                'vertical_tail': right_vertical_tail,
                'wheels': wheels + wheels,
                'fuselage': fuselage,
                'skids': skid,
                'propeller': propeller}
        # return self.find_children(lambda o: isinstance(o, (LoftedSolid,
        #                                                    RevolvedSolid,
        #                                                    RotatedShape)))

    # BE CAREFUL! THE MASS OF PAYLOAD AND BATTERY STILL HAS TO BE ADDED!
    @Attribute
    def center_of_gravity_of_components(self):
        dictionary = {}
        for component in self.pav_components:
            if type(self.pav_components[component]) is not list:
                name = component
                value = [self.pav_components[component].cog.x,
                         self.pav_components[component].cog.y,
                         self.pav_components[component].cog.z]
                if component == 'main_wing' or 'horizontal_tail' \
                        or 'vertical_tail':
                    value[1] = 0
                library = {name: value}
                dictionary = {**dictionary, **library}
            else:
                values = []
                for index in range(len(self.pav_components[component])):
                    value = [self.pav_components[component][index].cog.x,
                             self.pav_components[component][index].cog.y,
                             self.pav_components[component][index].cog.z, ]
                    if component == 'wheels' \
                            and index >= len(
                        self.pav_components[component]) / 2:
                        value[1] = - value[1]
                    values.append(value)
                name = component
                library = {name: values}
                dictionary = {**dictionary, **library}

        return dictionary

    @Attribute
    def mass_of_components(self):
        horizontal_tail_t_over_c = float(
            self.horizontal_tail.airfoils[0]) / 100
        vertical_tail_t_over_c = float(
            self.vertical_tail[0].airfoils[0]) / 100

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
                         * (self.pav_components['fuselage'].area
                            / (ft_to_m ** 2))
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
        mass_landing_gear = 20

        return {'main_wing': mass_wing,
                'horizontal_tail': mass_horizontal_tail,
                'vertical_tail': mass_vertical_tail,
                'wheels': mass_landing_gear,
                'fuselage': mass_fuselage,
                'skids': 50,
                'propeller': 40}

        # return [mass_wing, mass_fuselage,
        #         mass_horizontal_tail, mass_vertical_tail, mass_landing_gear]

    @Attribute
    def mass(self):
        mass = 0
        for component in self.center_of_gravity_of_components:
            if type(self.pav_components[component]) is not list:
                mass += self.mass_of_components[component]
            else:
                mass += (self.mass_of_components[component]
                         * len(self.center_of_gravity_of_components[
                                   component]))
        return mass

    @Attribute
    def centre_of_gravity(self):
        count = 0
        x_value = 0
        y_value = 0
        z_value = 0
        for component in self.center_of_gravity_of_components:
            if type(self.pav_components[component]) is not list:
                count += 1
                x_value += (self.center_of_gravity_of_components[component][0]
                            * self.mass_of_components[component])
                y_value += (self.center_of_gravity_of_components[component][1]
                            * self.mass_of_components[component])
                z_value += (self.center_of_gravity_of_components[component][2]
                            * self.mass_of_components[component])
            else:
                for index in range(len(self.pav_components[component])):
                    count += 1
                    x_value += (self.center_of_gravity_of_components[
                                    component][index][0]
                                * self.mass_of_components[component])
                    y_value += (self.center_of_gravity_of_components[
                                    component][index][1]
                                * self.mass_of_components[component])
                    z_value += (self.center_of_gravity_of_components[
                                    component][index][2]
                                * self.mass_of_components[component])
        return [x_value / self.mass,
                y_value / self.mass,
                z_value / self.mass]

    # -------------------------------------------------------------------------
    # Battery related
    # -------------------------------------------------------------------------

    @Attribute
    def propulsive_efficiency(self):
        return 0.9

    @Attribute
    def friction_drag_coefficient(self):
        return 0.02

    @Attribute
    def induced_drag_coefficient(self):
        analysis = AvlAnalysis(aircraft=self,
                               case_settings=cases)
        return analysis.induced_drag[cases[0][0]]

    @Attribute
    def total_drag_coefficient(self):
        return self.friction_drag_coefficient + self.induced_drag_coefficient

    @Attribute
    def battery_energy_density(self):
        # 200 Wh per kg converted to Joule per kg
        return 200 * 3600

    @Attribute
    def battery_discharge_time(self):
        # The factor 3/2 is included to compensate for slower flight phases
        # such as take-off and approach
        return self.range * 1000 / self.velocity * 3 / 2

    @Attribute
    def battery_power(self):
        return (0.5 * self.cruise_density * self.velocity ** 3 * self.wing_area
                * self.total_drag_coefficient / self.propulsive_efficiency)

    @Attribute
    def battery_mass(self):
        return (self.battery_power * self.battery_discharge_time /
                self.battery_energy_density)

    # -------------------------------------------------------------------------
    # Propeller related
    # -------------------------------------------------------------------------

    n_blades = Input(4)  # between 2 and 5
    r_rotor = Input(.4)
    roc_vertical = Input(10)  # between 5 and 15 m/s is most common

    #it now computes the number of rotors needed based on a given rotor diameter

    @Attribute
    def n_rotors(self):
        n_rotors_computed = (self.roc_vertical / (self.power_roc +
            self.power_d_liftingsurface) * (1500. * pi * self.r_rotor ** 2 -
            (self.power_hover + self.power_profile) / self.roc_vertical))
        n_rotors_per_side = ceil(n_rotors_computed / 2)
        return n_rotors_per_side * 2

    @Attribute
    def power_climb(self):
        return (self.power_hover + self.power_roc + self.power_profile +
                self.power_d_liftingsurface)

    @Attribute
    def power_hover(self):
        return self.thrust_hover * self.v_induced_climb

    @Attribute
    def thrust_hover(self):
        return (1. / 6. * self.n_blades * 6.6 * c_t / sigma_rotor
                * self.cruise_density * self.r_rotor / r_over_chord_rotor
                * (0.97 * mach_number_tip * (
                    sqrt(R * gamma * self.cruise_temperature))) ** 2
                * 0.97 * self.r_rotor * self.n_rotors)

    @Attribute
    def v_induced_climb(self):
        return (self.roc_vertical / 2
                + sqrt((self.roc_vertical / 2) ** 2 * self.thrust_hover
                       / (2 * self.cruise_density * pi * self.r_rotor)))

    @Attribute
    def power_roc(self):
        return self.maximum_take_off_weight * self.roc_vertical / 2.

    @Attribute
    def power_profile(self):
        return (self.c_d_rotor * 1. / 8. * self.cruise_density *
                self.r_rotor / r_over_chord_rotor * self.n_blades
                * (mach_number_tip * (sqrt(R * gamma * self.cruise_temperature))) ** 3
                * self.r_rotor * self.n_rotors)

    @Attribute
    def c_d_rotor(self):
        return (8. / (self.n_blades * self.r_rotor ** 2 / r_over_chord_rotor
                / (pi * self.r_rotor ** 2)) * sqrt(c_t / 2.)
                * (c_t / figure_of_merit - k_factor_rotor_drag * c_t))

    @Attribute
    def power_d_liftingsurface(self):
        return self.vertical_drag_liftingsurfaces * self.roc_vertical

    @Attribute
    def vertical_drag_liftingsurfaces(self):
        return (1. / 2. * self.cruise_density * self.roc_vertical ** 2
                * self.c_d_flatplate
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

    # HOW DO WE TREAT THE WEIGHT ESTIMATIONS?

    @Attribute
    def expected_maximum_take_off_weight(self):
        return ((self.mass + self.battery_mass
                 + self.number_of_passengers * 70) * g)

    @Input
    def maximum_take_off_weight(self):
        # MTOW in Newtons
        return ((3000 + 500
                 + self.number_of_passengers * 70) * g)

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

    # -------------------------------------------------------------------------
    # Tail related
    # -------------------------------------------------------------------------

    # ADJUST THIS THING !!!!!!!!!!
    @Attribute
    def v_horizontal_tail(self):
        return self.cruise_velocity * 0.85



    @Attribute
    def horizontal_tail_area(self):
        return 3

    @Attribute
    def vertical_tail_area(self):
        return 2

    # ADJUST THIS THING !!!!!!!!!!
    @Attribute
    def horizontal_tail_sweep(self):
        return 20

    @Attribute
    def vertical_tail_sweep(self):
        root_position = self.vertical_tail_root_location
        tip_position = (self.horizontal_tail.position.x
                        + self.lateral_position_of_skids
                        * tan(radians(self.horizontal_tail_sweep)))
        return degrees(atan((tip_position - root_position)
                            / self.vertical_tail_span))

    @Attribute
    def vertical_tail_tip_chord(self):
        return chord_length(self.horizontal_tail.root_chord,
                            self.horizontal_tail.tip_chord,
                            self.lateral_position_of_skids
                            / (self.horizontal_tail.span / 2))

    @Attribute
    def vertical_tail_span(self):
        return (self.horizontal_tail.position.z
                + self.lateral_position_of_skids
                * tan(radians(self.horizontal_tail.dihedral))
                - self.vertical_position_of_skids)

    @Attribute
    def vertical_tail_root_chord(self):
        return (2 * (self.vertical_tail_area / 2)
                / self.vertical_tail_span - self.vertical_tail_tip_chord)

    @Attribute
    def vertical_tail_taper_ratio(self):
        return self.vertical_tail_tip_chord / self.vertical_tail_root_chord

    @Attribute
    def vertical_tail_aspect_ratio(self):
        return (self.vertical_tail_span /
                (self.vertical_tail_root_chord *
                 (1 + self.vertical_tail_taper_ratio) / 2))

    @Attribute
    def vertical_tail_root_location(self):
        # This provides the location relative to the nose
        longitudinal = (self.horizontal_tail.position.x
                        + self.lateral_position_of_skids
                        * tan(radians(self.horizontal_tail_sweep))
                        )
        return longitudinal

    # -------------------------------------------------------------------------
    # Propellers
    # -------------------------------------------------------------------------


    @Attribute
    def number_of_propellers(self):
        return ceil(self.wing_area / 2)

    @Input
    def number_of_vtol_propellers(self):
        return 12

    @Attribute
    def propeller_radii(self):
        return ([1 - 0.1 * self.number_of_propellers]
                * self.number_of_propellers)

    @Input
    def vtol_propeller_radius(self):
        return 6 / self.number_of_vtol_propellers

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
    def vtol_propeller_locations(self):
        front_propellers = [self.longitudinal_position_of_skids
                            + (1 + 2 * prop) * self.prop_separation_factor
                            * self.vtol_propeller_radius
                            for prop in range(self.length_of_skids[2])]
        centre_propellers = [self.front_connection_location.x
                             + self.front_connection_chord * 3 / 4
                             + (0.5 + prop) * self.length_of_skids[1]
                             / self.length_of_skids[3]
                             for prop in range(self.length_of_skids[3])]
        rear_propellers = [self.vertical_tail_root_location
                           + self.vertical_tail_root_chord * 3 / 4
                           * self.prop_separation_factor
                           + (1 + 2 * prop) * self.prop_separation_factor
                           * self.vtol_propeller_radius
                           for prop in range(self.length_of_skids[4])]
        longitudinal_position_per_side = (front_propellers + centre_propellers
                                          + rear_propellers)
        longitudinal_position = (longitudinal_position_per_side
                                 + longitudinal_position_per_side)
        lateral_position_right = ([self.lateral_position_of_skids]
                                  * int(self.number_of_vtol_propellers / 2.))
        lateral_position_left = ([- self.lateral_position_of_skids]
                                 * int(self.number_of_vtol_propellers / 2.))
        lateral_position = lateral_position_left + lateral_position_right
        vertical_position = ([self.vertical_position_of_skids]
                             * self.number_of_vtol_propellers)
        return [translate(self.position,
                          self.position.Vx,
                          longitudinal_position[index],
                          self.position.Vy,
                          lateral_position[index],
                          self.position.Vz,
                          vertical_position[index])
                for index in range(self.number_of_vtol_propellers)]

    # -------------------------------------------------------------------------
    # Skids
    # -------------------------------------------------------------------------

    @Attribute
    def prop_separation_factor(self):
        return 1.2

    # ADJUST LATER:
    @Attribute
    def length_of_skids(self):
        # Check the distance between the front connection and vertical tail
        # if there can be propellers placed in between
        distance_between_connections = (self.vertical_tail_root_location
                                        - self.vertical_tail_root_chord / 4
                                        - (self.front_connection_location.x
                                           + self.front_connection_chord
                                           * 3 / 4))
        # Start with zero propellers and increase the number until it
        # doesn't fit anymore
        number_of_props_in_middle = 1
        while ((distance_between_connections
                / (number_of_props_in_middle
                   * (self.vtol_propeller_radius * 2)))
               > self.prop_separation_factor):
            number_of_props_in_middle += 1
        # Make sure that all propellers fit, but use only those that are
        # required if this number is smaller than what fits in between
        usable_number_of_props = min(number_of_props_in_middle - 1,
                                     self.number_of_vtol_propellers / 2)
        # Compute how many propellers per skid need to be placed outside the
        # central part
        remaining_number_of_props = (self.number_of_vtol_propellers / 2
                                     - usable_number_of_props)
        # Define the number of propellers per skid that needs to be placed
        # ahead of the front connection
        props_on_the_front = ceil(remaining_number_of_props / 2)
        props_on_the_rear = remaining_number_of_props - props_on_the_front
        length = (distance_between_connections
                  + (self.front_connection_chord +
                     self.vertical_tail_root_chord)
                  * self.prop_separation_factor
                  + remaining_number_of_props * self.vtol_propeller_radius
                  * 2 * self.prop_separation_factor)
        # Provide as output the length of the skid, as well as how the
        # propellers are divided, such that they can be positioned later on
        return [length, distance_between_connections, int(props_on_the_front),
                int(usable_number_of_props), int(props_on_the_rear)]

    @Attribute
    def vertical_skid_profile(self):
        return '0018'

    # @Attribute
    # def vertical_skid_chord(self):
    #     return 0.75

    @Attribute
    def skid_height(self):
        return min(0.1, 0.9 * self.skid_width)

    @Attribute
    def skid_width(self):
        return (1.05 * self.vertical_tail_root_chord *
                (float(self.vertical_skid_profile) / 100))

        # ADJUST LATER

    @Attribute
    def longitudinal_position_of_skids(self):
        return (self.front_connection_location.x
                # - self.front_connection_chord / 2
                # * self.prop_separation_factor
                - (0.5 + self.length_of_skids[2])
                * self.vtol_propeller_radius * 2)

    @Attribute
    def lateral_position_of_skids(self):
        return 0.7 * self.cabin_width + self.vtol_propeller_radius

    @Attribute
    def vertical_position_of_skids(self):
        return (- (0.5 + 0.2) * self.cabin_height
                if self.wheels_included is False
                else - (0.5 + 0.2) * self.cabin_height
                     + self.wheel_radius + self.vertical_rod_length)

    @Attribute
    def skid_locations(self):
        return [translate(self.position,
                          self.position.Vx,
                          self.longitudinal_position_of_skids,
                          self.position.Vy,
                          - self.lateral_position_of_skids
                          + 2 * self.lateral_position_of_skids * index,
                          self.position.Vz,
                          self.vertical_position_of_skids)
                for index in range(2)]

    # Used to subtract the propeller cones from the skids
    @Attribute
    def arrange_skids(self):
        first_skid = [self.vtol_propellers[index].hub_cone
                      for index in range(int(self.number_of_vtol_propellers
                                             / 2))]
        second_skid = [self.vtol_propellers[index].hub_cone
                       for index in range(int(self.number_of_vtol_propellers
                                              / 2),
                                          self.number_of_vtol_propellers)]
        return [first_skid, second_skid]

    @Attribute
    def arrange_struts(self):
        first_skid = [self.left_wheel_rods[index].rod_vertical
                      for index in range(int(self.number_of_wheels
                                             / 2))]
        second_skid = [self.right_wheel_rods[index - int(
            self.number_of_wheels / 2)]
                       for index in range(int(self.number_of_wheels
                                              / 2),
                                          self.number_of_wheels)]
        return [first_skid, second_skid]

    # -------------------------------------------------------------------------
    # Connections
    # -------------------------------------------------------------------------

    @Attribute
    def front_connection_chord(self):
        return (self.skid_height * 0.9
                / (float(self.vertical_skid_profile) / 100))

    @Attribute
    def front_connection_location(self):
        return translate(self.position,
                         self.position.Vx,
                         self.length_of_fuselage_nose
                         - self.front_connection_chord / 4,
                         self.position.Vz,
                         (2 * self.fuselage.nose_height - 1)
                         * self.cabin_height / 6)

    @Attribute
    def front_connection_vertical_length(self):
        return (self.vertical_position_of_skids -
                self.front_connection_location.z)

    @Attribute
    def front_connection_horizontal_length(self):
        return (self.lateral_position_of_skids -
                self.front_connection_location.y)

    @Attribute
    def front_connection_span(self):
        return self.front_connection_horizontal_length

    @Attribute
    def front_connection_dihedral(self):
        return degrees(atan(self.front_connection_vertical_length /
                            self.front_connection_horizontal_length))

    # -------------------------------------------------------------------------
    # Wheels
    # -------------------------------------------------------------------------

    # ADJUST THIS LATER

    @Attribute
    def number_of_wheels(self):
        expected_number_of_wheels = self.wing_aspect_ratio  # Adjust this!!!!
        number_of_wheels = (2 * ceil(expected_number_of_wheels / 2)
                            if expected_number_of_wheels >= 4 else 4)
        return number_of_wheels

    @Attribute
    def wheel_locations(self):
        wheels_per_side = (int(self.number_of_wheels / 2)
                           if self.number_of_wheels * self.wheel_radius
                              < 0.8 * self.length_of_skids[0]
                           else ceil(0.8 * self.length_of_skids[0] /
                                     (2 * self.wheel_radius)))
        left_locations = [translate(self.skid_locations[0],
                                    self.position.Vx,
                                    (index + 0.5) / wheels_per_side
                                    * self.length_of_skids[0],
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

    # -------------------------------------------------------------------------
    # AVL
    # -------------------------------------------------------------------------

    @Attribute
    def avl_surfaces(self):
        return self.find_children(lambda o: isinstance(o, avl.Surface))

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    # Show a point indicating the c.g.
    @Part
    def center_of_gravity_point(self):
        return Point(x=self.centre_of_gravity[0],
                     y=self.centre_of_gravity[1],
                     z=self.centre_of_gravity[2])

    @Part
    def main_wing(self):
        return LiftingSurface(name='main_wing',
                              number_of_profiles=4,
                              airfoils=['34018', '34015', 'whitcomb', '43008'],
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
                              span=sqrt(self.wing_aspect_ratio
                                        * self.horizontal_tail_area),
                              aspect_ratio=self.wing_aspect_ratio * 0.7,
                              taper_ratio=0.4,
                              sweep=self.horizontal_tail_sweep,
                              incidence_angle=0,
                              twist=0,
                              dihedral=3,
                              position=self.position.translate(
                                  self.position.Vx, self.fuselage_length * 0.8,
                                  self.position.Vz, 0.3 * self.cabin_height),
                              color='silver')

    @Part
    def vertical_tail(self):
        return LiftingSurface(name='vertical_tails',
                              quantify=len(self.skid_locations),
                              number_of_profiles=2,
                              airfoils=[self.vertical_skid_profile],
                              is_mirrored=False,
                              # Connect the skid to the horizontal tail
                              span=self.vertical_tail_span,
                              aspect_ratio=self.vertical_tail_aspect_ratio,
                              taper_ratio=self.vertical_tail_taper_ratio,
                              sweep=self.vertical_tail_sweep,
                              incidence_angle=0,
                              twist=0,
                              dihedral=0,
                              position=rotate90(
                                  translate(self.position,
                                            self.position.Vx,
                                            self.vertical_tail_root_location,
                                            self.position.Vy,
                                            self.lateral_position_of_skids
                                            * (-1 + 2 * child.index),
                                            self.position.Vz,
                                            self.vertical_position_of_skids),
                                  self.position.Vx),
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

    # -------------------------------------------------------------------------
    # PROPELLERS
    # -------------------------------------------------------------------------

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
                         aspect_ratio=7,
                         ratio_hub_to_blade_radius=0.15,
                         leading_edge_sweep=0,
                         blade_setting_angle=40,
                         blade_outwash=30,
                         number_of_blade_sections=40,
                         blade_thickness=60,
                         position=rotate90(
                             self.propeller_locations[child.index],
                             - self.position.Vy),
                         color=self.secondary_colour)

    @Part
    def vtol_propellers(self):
        return Propeller(name='VTOL_propellers',
                         quantify=self.number_of_vtol_propellers,
                         number_of_blades=4,
                         blade_radius=self.vtol_propeller_radius,
                         hub_length=1.5 * self.skid_height,
                         nacelle_included=False,
                         aspect_ratio=6,
                         ratio_hub_to_blade_radius=
                         min(0.2, 0.9 * (self.skid_width / 2)
                             / self.vtol_propeller_radius),
                         leading_edge_sweep=0,
                         blade_setting_angle=30,
                         blade_outwash=25,
                         number_of_blade_sections=40,
                         blade_thickness=50,
                         position=self.vtol_propeller_locations[child.index],
                         color=self.secondary_colour)

    # @Part
    # def vtol_hubs(self):
    #     return SubtractedSolid(quantify=self.number_of_vtol_propellers,
    #                            shape_in=self.vtol_propellers_reference[
    #                                child.index].hub_cone,
    #                            tool=
    #                            self.skids[round(
    #                                child.index
    #                                / self.number_of_vtol_propellers)].skid)
    #
    # @Part
    # def vtol_propellers(self):
    #     return Propeller(name='VTOL_propellers',
    #                      quantify=self.number_of_vtol_propellers,
    #                      number_of_blades=4,
    #                      blade_radius=self.vtol_propeller_radius,
    #                      hub_length=1.5 * self.skid_height,
    #                      hide_hub=True,
    #                      nacelle_included=False,
    #                      aspect_ratio=6,
    #                      ratio_hub_to_blade_radius=
    #                      min(0.2, 0.9 * (self.skid_width / 2)
    #                          / self.vtol_propeller_radius),
    #                      leading_edge_sweep=0,
    #                      blade_setting_angle=30,
    #                      blade_outwash=25,
    #                      number_of_blade_sections=40,
    #                      blade_thickness=50,
    #                      position=self.vtol_propeller_locations[child.index],
    #                      color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # SKIDS
    # -------------------------------------------------------------------------

    @Part(in_tree=False)
    def skids(self):
        return Skid(quantify=2,
                    color=self.secondary_colour,
                    skid_length=self.length_of_skids[0],
                    skid_width=self.skid_width,
                    skid_height=self.skid_height,
                    # skid_connection_profile=self.vertical_skid_profile,
                    # chord_skid_connection=self.vertical_skid_chord,
                    # # Connect the skid to the horizontal tail
                    # span_skid_connection=
                    # self.horizontal_tail.position.z
                    # + abs(self.skid_locations[child.index].y)
                    # * tan(radians(self.horizontal_tail.dihedral))
                    # - self.skid_locations[child.index].z,
                    position=self.skid_locations[child.index])

    @Part
    def landing_skids(self):
        return SubtractedSolid(quantify=2,
                               shape_in=self.skids[child.index].skid,
                               tool=self.arrange_skids[child.index])
        # self.arrange_struts[child.index]])

    @Part
    def right_front_connection(self):
        return LiftingSurface(name='front_connections',
                              number_of_profiles=2,
                              airfoils=[self.vertical_skid_profile],
                              is_mirrored=False,
                              span=self.front_connection_span,
                              aspect_ratio=self.front_connection_span
                                           / self.front_connection_chord,
                              taper_ratio=1,
                              sweep=0,
                              incidence_angle=0,
                              twist=0,
                              dihedral=self.front_connection_dihedral,
                              position=self.front_connection_location,
                              color=self.secondary_colour)

    @Part
    def left_front_connection(self):
        return MirroredShape(shape_in=self.right_front_connection.surface,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color=self.secondary_colour)

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

    #
    # @Part
    # def left_struts(self):
    #     return SubtractedSolid(quantify=len(self.wheel_locations),
    #                            shape_in=[self.left_wheel_rods[
    #                                child.index].rods.rod_horizontal,
    #                                      self.left_wheel_rods[
    #                                          child.index].rods.rod_vertical],
    #                            tool=self.skids[0].skid)

    @Part
    def right_wheel_rods(self):
        return MirroredShape(quantify=len(self.wheel_locations),
                             shape_in=self.left_wheel_rods[child.index],
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
