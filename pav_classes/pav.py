"""
This file contains the complete personal aerial vehicle class. It is
structured in the following way:

- Outside the class:

    - Imports and paths
    - Constants
    - Functions

- Inside the class:

    - Inputs
    - Input checks
    - Flight conditions
    - Flight performance
    - Battery
    - Mass
    - Fuselage
    - Wing
    - Horizontal tail
    - Vertical tail
    - Skids
    - Skid connections
    - Wheels
    - Cruise propellers
    - VTOL rotors
    - Interfaces:

        - AVL
        - STEP

Note that the attributes and parts are both placed in the relevant sections,
i.e. the fuselage part is placed below the fuselage related attributes,
and above the wing related attributes.
"""

# -----------------------------------------------------------------------------
# IMPORTS AND PATHS
# -----------------------------------------------------------------------------

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
from .functions import *

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')
FILENAME = os.path.join(OUTPUT_DIR, 'pav_assembly.stp', '')

# -----------------------------------------------------------------------------
# CONSTANTS
# -----------------------------------------------------------------------------

g = 9.80665
gamma = 1.4
R = 287

lbs_to_kg = 0.45359237
ft_to_m = 0.3048
inch_to_m = 0.0254

c_t = 0.0050  # between 0.0050 and 0.0060
c_t_cruise = 0.10  # between 0.02 and 0.16
sigma_rotor = 0.070  # fixed
mach_number_tip = 0.6  # fixed, higher Mach numbers for the rotor tips lead
# to stall
dl_max = 1500  # fixed
r_over_chord_rotor: int = 15  # between 15 and 20
figure_of_merit = 0.8  # between 0.6 and 0.8
twist_rotor = -10  # degrees, tip angle is 10 degrees lower than at root,
# which is good for hover
k_factor_rotor_drag = 1.15  # between 1.1 and 1.2

design_lift_coefficient = 0.5

# -----------------------------------------------------------------------------
# FUNCTIONS
# -----------------------------------------------------------------------------

cases = [('fixed_cl',
          {'alpha': avl.Parameter(name='alpha',
                                  value=str(design_lift_coefficient),
                                  setting='CL')})]


# -----------------------------------------------------------------------------
# PAV
# -----------------------------------------------------------------------------


class PAV(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

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
    cruise_velocity = Input(400)
    # The colours that are used for the visualisation
    primary_colour = Input('white')
    secondary_colour = Input('red')

    # MAKE THESE FIXED ATTRIBUTES LATER ON?

    n_blades = Input(4)  # between 2 and 5
    r_rotor = Input(.4)
    roc_vertical = Input(10)  # between 5 and 15 m/s is most common
    c_d_flatplate = Input(1.28)
    r_propeller = Input(0.3)


    @Input
    def design_cl(self):
        return design_lift_coefficient

    @Input
    def cruise_altitude_in_feet(self):
        return 10e3

    # -------------------------------------------------------------------------
    # INPUT CHECKS
    # -------------------------------------------------------------------------

    @Attribute
    def velocity(self):
        velocity = self.cruise_velocity / 3.6
        if velocity / self.cruise_speed_of_sound > 0.6:
            message = 'The cruise velocity is set too high. The cruise ' \
                      'velocity will be set to' \
                      '{} km/h.'.format(int(3.6 * 0.6
                                            * self.cruise_speed_of_sound))
            generate_warning('Warning: value changed', message)
            return 0.6 * self.cruise_speed_of_sound
        else:
            return velocity

    # -------------------------------------------------------------------------
    # FLIGHT CONDITIONS
    # -------------------------------------------------------------------------

    # Atmospheric conditions during cruise

    @Attribute
    def cruise_altitude(self):
        # Convert the input altitude in feet to metres
        return self.cruise_altitude_in_feet * 0.3048

    @Attribute
    def cruise_temperature(self):
        # Use a lapse rate of -6.5 K per km
        return 288.15 - 0.0065 * self.cruise_altitude

    @Attribute
    def cruise_density(self):
        # Use a reference density of 1.225 kg/m^3 at sea level
        return (1.225 * (self.cruise_temperature / 288.15)
                ** (-1 - g / (R * -0.0065)))

    @Attribute
    def kinematic_viscosity_air(self):
        temperature_rankine = self.cruise_temperature * 9. / 5.
        absolute_viscosity = (3.62 * 10 ** -7
                              * (temperature_rankine / 518.7) ** 1.5
                              * (518.7 + 198.72)
                              / (temperature_rankine + 198.72))
        return absolute_viscosity * 47.88 / self.cruise_density

    # Flight velocity related to cruise conditions

    @Attribute
    def cruise_speed_of_sound(self):
        return sqrt(gamma * R * self.cruise_temperature)

    @Attribute
    def cruise_mach_number(self):
        return self.velocity / self.cruise_speed_of_sound

    # -------------------------------------------------------------------------
    # FLIGHT PERFORMANCE
    # -------------------------------------------------------------------------

    @Attribute
    def propulsive_efficiency(self):
        return 0.9

    @Attribute
    def friction_drag_coefficient(self):
        # The estimated CD0 for the entire vehicle based on references
        return 0.02

    @Attribute
    def induced_drag_coefficient(self):
        # Obtain the induced drag from the AVL analysis
        analysis = AvlAnalysis(aircraft=self,
                               case_settings=cases)
        # print(analysis.induced_drag[cases[0][0]])
        return analysis.induced_drag[cases[0][0]]

    @Attribute
    def total_drag_coefficient(self):
        # Total drag is a combination of friction drag and induced drag
        return self.friction_drag_coefficient + self.induced_drag_coefficient

    # -------------------------------------------------------------------------
    # BATTERY
    # -------------------------------------------------------------------------

    @Attribute
    def battery_energy_density(self):
        # 200 Wh per kg converted to Joule per kg
        return 200 * 3600

    @Attribute
    def battery_discharge_time(self):
        # The factor 3/2 is included to compensate for slower flight phases
        # such as take-off and approach, as well as diversion
        return self.range * 1000 / self.velocity * 3 / 2

    @Attribute
    def battery_power(self):
        # The required power is equal to the total drag * velocity, divided
        # by the propulsive efficiency
        return (0.5 * self.cruise_density * self.velocity ** 3 * self.wing_area
                * self.total_drag_coefficient / self.propulsive_efficiency)

    @Attribute
    def battery_mass(self):
        # The battery mass depends on the battery energy, which is the
        # battery power * battery discharge time
        return (self.battery_power * self.battery_discharge_time /
                self.battery_energy_density)

    # -------------------------------------------------------------------------
    # MASS
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
                'propeller': propeller,
                'battery': 0}
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
                if name == 'battery':
                    value = [0.25 * self.fuselage_length, 0, 0]
                else:
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

        return {'main_wing': 40 * self.wing_area,
                'horizontal_tail': 40 * self.horizontal_tail_area,
                'vertical_tail': 40 * self.vertical_tail_area,
                'wheels': mass_landing_gear,
                'fuselage': 50 * self.fuselage_length,
                'skids': 25,
                'propeller': 15,
                'battery': self.battery_mass}

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
    def centre_of_gravity_result(self):
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

    @Input
    def centre_of_gravity(self):
        return [2., 0, 0.1]

        # HOW DO WE TREAT THE WEIGHT ESTIMATIONS?

    @Attribute
    def expected_maximum_take_off_weight(self):
        return ((self.mass
                 + self.number_of_passengers * 70) * g)

    @Input
    def maximum_take_off_weight(self):
        fr = 1.5 + (self.range - 100) * 1e3 * 0.0025 / 1e3
        fv = 1.5 + (self.velocity - 100) * 0.0025
        # MTOW in Newtons
        return 3.5 * fr * fv * (self.number_of_passengers * 70) * g

    # Show a point indicating the c.g.
    @Part
    def center_of_gravity_point(self):
        return Point(x=self.centre_of_gravity[0],
                     y=self.centre_of_gravity[1],
                     z=self.centre_of_gravity[2])

    # -------------------------------------------------------------------------
    # FUSELAGE
    # -------------------------------------------------------------------------

    # Attributes related to the passenger cabin

    @Attribute
    def seat_pitch(self):
        # 1.4 m for business class and 0.95 m for economy class
        return 1.4 if self.quality_level == 2 else 0.95

    @Attribute
    def seat_width(self):
        # 0.7 m for business class and 0.5 m for economy class
        return 0.7 if self.quality_level == 2 else 0.5

    @Attribute
    def number_of_seats_abreast(self):
        # If the PAV is not a one-seater, the number of seats abreast is 2
        # if there are 8 or less passengers; if there are more than 8
        # passengers, use 3 seats per row
        return 1 if self.number_of_passengers < 4 else 2

    @Attribute
    def number_of_rows(self):
        # The number of rows should always allow at least all the number of
        # seats and then round up
        return ceil(self.number_of_passengers
                    / self.number_of_seats_abreast)

    # Attributes related to the outer dimensions of the fuselage

    @Attribute
    def cabin_width(self):
        # Add 20 cm to the cabin width to account for additional things; this
        # assumes that there is no aisle, but each row has its own exits
        return self.seat_width * self.number_of_seats_abreast + 0.2

    @Attribute
    def cabin_height(self):
        # The cabin height is generally the same as the width, except if the
        # width is smaller than 1.6 m; then a minimum height of 1.6 m is
        # established
        return self.cabin_width if self.cabin_width >= 1.6 else 1.6

    @Input
    def length_of_fuselage_nose(self):
        # The length of the nose cone depends on the number of passengers
        return (1 if self.number_of_passengers <= 4
                else 0.6 * self.number_of_seats_abreast)

    @Attribute
    def cabin_length(self):
        # Allow the cabin to be placed in the aft 1/4 th of the fuselage
        # nose cone
        return (self.number_of_rows * self.seat_pitch
                - self.length_of_fuselage_nose / 4)

    @Input
    def length_of_fuselage_tail(self):
        # The length of the tail cone depends on the number of passengers
        return (1.5 if self.number_of_passengers <= 4
                else 1 + self.number_of_seats_abreast / 2)

    @Attribute
    def fuselage_length(self):
        # Sum the lengths of the nose cone, cabin and tail cone
        return (self.length_of_fuselage_nose + self.cabin_length +
                self.length_of_fuselage_tail)

    # The fuselage part, based on the attributes above

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
                        door_height=self.cabin_height-0.15,
                        nose_height=-0.2,
                        tail_height=0.3,
                        color=self.primary_colour)

    # -------------------------------------------------------------------------
    # WING
    # -------------------------------------------------------------------------

    # Geometric aspects of the wing

    @Attribute
    def wing_area(self):
        # Required wing area for cruise
        area = (self.maximum_take_off_weight
                / (0.5 * self.cruise_density * self.velocity ** 2
                   * self.design_cl))
        # print(area)
        return area

    @Attribute
    def intended_wing_aspect_ratio(self):
        # This is the aspect ratio that would be obtained if the wing is not
        # span-limited
        return 10

    @Attribute
    def wing_span(self):
        # The span can be limited by setting a maximum span
        span = sqrt(self.intended_wing_aspect_ratio * self.wing_area)
        return span if span < self.maximum_span else self.maximum_span

    @Attribute
    def wing_aspect_ratio(self):
        # Define the aspect ratio: A = b^2 / S
        return self.wing_span ** 2 / self.wing_area

    @Attribute
    def wing_sweep(self):
        # Below Mach 0.4, no sweep is applied. For Mach numbers between 0.4
        # and 0.6, linear interpolation is applied until the wing sweep is
        # 10 degrees at the quarter chord for Mach 0.6; this is the maximum
        # Mach number for the PAV
        return (0 if self.cruise_mach_number < 0.4 else
                (self.cruise_mach_number - 0.4) * 50)

    @Attribute
    def wing_dihedral(self):
        # For a high wing configuration, the dihedral is set to 3 degrees;
        # for low wing configurations, dihedral is set to 1 degree
        return 3 if self.wing_location.z > 0 else 1

    # Position of the wing

    @Input
    def longitudinal_wing_position(self):
        # This input is used to iterate for wing positioning
        return 0.4

    @Attribute
    def wing_location(self):
        # The length ratio is changed as per input for iterations
        length_ratio = self.longitudinal_wing_position
        # The wing is positioned at 90% of the cabin height
        height_ratio = 0.9
        return self.position.translate(self.position.Vx,
                                       length_ratio * self.fuselage_length,
                                       self.position.Vz,
                                       (height_ratio - 0.5)
                                       * self.cabin_height)

    # The wing parts, based on the attributes above; the main_wing is used
    # as a reference, while the right_wing and left_wing are visible in the GUI

    @Part(in_tree=False)
    def main_wing(self):
        return LiftingSurface(name='main_wing',
                              number_of_profiles=4,
                              airfoils=['24018', '24015', '24012', '24010'],
                              is_mirrored=True,
                              span=self.wing_span,
                              aspect_ratio=self.wing_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep,
                              incidence_angle=1,
                              twist=-2,
                              dihedral=self.wing_dihedral,
                              position=self.wing_location,
                              color='silver')

    @Part
    def right_wing(self):
        return SubtractedSolid(shape_in=self.main_wing.surface,
                               tool=self.fuselage.fuselage_shape,
                               color='silver')

    @Part
    def left_wing(self):
        return MirroredShape(shape_in=self.right_wing,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='silver')

    # -------------------------------------------------------------------------
    # HORIZONTAL TAIL
    # -------------------------------------------------------------------------

    # Flight conditions specifically related to the horizontal tail

    @Attribute
    def cruise_velocity_horizontal_tail(self):
        # The perturbing presence of the fuselage reduces the flow investing
        # the horizontal tail compared to the flight velocity
        return self.velocity * 0.85

    # Geometric parameters related to the tail

    @Attribute
    def horizontal_tail_aspect_ratio(self):
        # Obtained from literature, the aspect ratio is approximately 5 for
        # jet aircraft
        return 5.

    @Attribute
    def horizontal_tail_sweep(self):
        # The sweep of the horizontal tail is generally 10 degrees more than
        # that of the wing
        return self.wing_sweep + 10

    # The longitudinal position of the wing leading edge at the intersection
    # point with the fuselage is required for performance parameters of the
    # tail

    @Attribute
    def wing_location_le(self):
        # First obtain the longitudinal position of the root quarter chord
        # point
        return (self.main_wing.position.x
                # Then add the distance due to sweep up to the edge of the
                # cabin
                + tan(radians(self.wing_sweep)) * self.cabin_width / 2
                # Subtract the local quarter chord length to obtain the
                # local leading edge position
                - chord_length(self.main_wing.root_chord,
                               self.main_wing.tip_chord,
                               self.cabin_width / (self.wing_span / 2)) / 4
                )

    @Attribute
    # Get the aerodynamic centre of the wing and fuselage combined
    def aerodynamic_center_wing_and_fuselage(self):
        # Define (x_ac / c), the non-dimensional aerodynamic centre of the
        # wing relative to the leading edge of the mean aerodynamic chord;
        # it can be assumed that this is 1/4 of the mean aerodynamic chord
        x_mac = (self.main_wing.mean_aerodynamic_chord / 4)
        # Define (x_ac / c)_wf, the non-dimensional aerodynamic centre of
        # the wing-fuselage combination, relative to the leading edge of the
        # mean aerodynamic chord
        x_position = (x_mac
                      # Get the relative x position of the fuselage
                      - 1.8 / self.lift_coefficient_alpha_wing_and_fuselage
                      * self.cabin_width * self.cabin_height
                      * self.wing_location_le
                      / (self.wing_area
                         * self.main_wing.mean_aerodynamic_chord)
                      + 0.273 / (1 + self.main_wing.taper_ratio)
                      * self.cabin_width * self.wing_area / self.wing_span
                      * (self.wing_span - self.cabin_width)
                      / (self.main_wing.mean_aerodynamic_chord ** 2
                         * (self.wing_span + 2.15 * self.cabin_width))
                      * tan(radians(self.wing_sweep)))
        # Return the non-dimensional position of the combined centre of
        # gravity, relative to the leading edge of the mean aerodynamic chord
        return x_position

    @Attribute
    def tail_arm(self):
        # It is assumed that the aerodynamic centre of the horizontal tail
        # is close to the quarter chord point of the root chord; the tail
        # arm is the position of this aerodynamic centre minus the position
        # of the aerodynamic centre of the wing
        return (self.horizontal_tail.position.x
                - (self.wing_location.x
                   + tan(radians(self.wing_sweep))
                   * self.main_wing.lateral_position_of_mean_aerodynamic_chord)
                )

    # Performance coefficients related to the wing

    @Attribute
    def lift_coefficient_alpha_wing(self):
        return (2. * pi * self.wing_aspect_ratio
                / (2.
                   + sqrt(4.
                          + (self.wing_aspect_ratio
                             * sqrt(1 - self.cruise_mach_number ** 2)
                             / 0.95) ** 2
                          * (1. + (tan(
                            # Obtain the half chord sweep from the quarter
                            # chord sweep
                            sweep_to_sweep(0.25, radians(self.wing_sweep),
                                           0.5,
                                           self.wing_aspect_ratio,
                                           self.main_wing.taper_ratio))
                                   / sqrt(
                                    1 - self.cruise_mach_number ** 2))
                             ** 2)
                          )
                   )
                )

    @Attribute
    def lift_coefficient_alpha_wing_and_fuselage(self):
        return (self.lift_coefficient_alpha_wing
                * (1 + 2.15 * self.cabin_width / self.wing_span)
                * (self.wing_area - self.cabin_width *
                   self.main_wing.root_chord)
                / self.wing_area + pi / 2. * self.cabin_width ** 2
                / self.wing_area)

    @Attribute
    def moment_coefficient_aerodynamic_centre(self):
        # First obtain the part due to the wing; -0.06 is the moment
        # coefficient of a reference airfoil at zero lift; it is assumed
        # that this approximation holds for the various airfoils that can be
        # used
        wing = (-0.06 * self.wing_aspect_ratio
                * (cos(radians(self.wing_sweep)) ** 2)
                / (self.wing_aspect_ratio + 2 * cos(
                    radians(self.wing_sweep))))
        # The contribution of the fuselage depends on the lift coefficient
        # at 0 degrees angle of attack; this ranges from 0.1 to 0.4, and the
        # value of 0.25 is used as a mean
        fuselage = (-1.8 * (
                1 - 2.5 * self.cabin_width / self.fuselage_length)
                    * (pi * self.cabin_width * self.cabin_height
                       * self.fuselage_length)
                    / (4 * self.wing_area
                       * self.main_wing.mean_aerodynamic_chord)
                    * 0.25 / self.design_cl)
        # Combine the two contributions of the wing and fuselage
        return wing + fuselage

    @Attribute
    def down_wash(self):
        # Obtain the longitudinal and vertical locations of the aerodynamic
        # centre of the wing
        wing_x = (self.main_wing.position.x
                  + tan(radians(self.wing_sweep))
                  * self.main_wing.lateral_position_of_mean_aerodynamic_chord)
        wing_z = (self.main_wing.position.z
                  + tan(radians(self.main_wing.dihedral))
                  * self.main_wing.lateral_position_of_mean_aerodynamic_chord)
        # Approximate the longitudinal and vertical locations of the
        # aerodynamic centre of the horizontal tail, assuming it is close to
        # the quarter chord point of the root chord
        h_t_x = self.horizontal_tail.position.x
        h_t_z = self.horizontal_tail.position.z
        # Obtain the distances between the wing and horizontal tail
        distance_wing_tail_x = abs(h_t_x - wing_x)
        distance_wing_tail_z = abs(wing_z - h_t_z)
        r = distance_wing_tail_x / (self.wing_span / 2)
        # Define the K epsilon terms accounting for the wing sweep angle effect
        k_epsilon_wing_sweep = ((0.1124 + 0.1265 * radians(self.wing_sweep)
                                 + 0.1766 * radians(self.wing_sweep) ** 2)
                                / (r ** 2)
                                + 0.1024 / r + 2.)
        k_epsilon_wing_zero_sweep = (0.1124 / (r ** 2)
                                     + 0.1024 / r + 2.)
        # Define the wing down wash gradient
        return (k_epsilon_wing_sweep / k_epsilon_wing_zero_sweep
                * (r / (r ** 2 + distance_wing_tail_z ** 2)
                   * 0.4876
                   / (sqrt(r ** 2 + 0.6319 + distance_wing_tail_z ** 2))
                   + (1 + (r ** 2 / (r ** 2 + 0.7915 + 5.0734 *
                                     distance_wing_tail_z ** 2)) ** 0.3113)
                   * (1 - sqrt(distance_wing_tail_z ** 2
                               / (1 + distance_wing_tail_z ** 2))
                      )
                   ) * self.lift_coefficient_alpha_wing
                / (pi * self.wing_aspect_ratio)
                )

    # Performance coefficients related to the horizontal tail

    @Attribute
    def horizontal_tail_lift_coefficient(self):
        return - 0.35 * self.horizontal_tail_aspect_ratio ** (1 / 3)

    @Attribute
    def lift_coefficient_alpha_horizontal_tail(self):
        # Compute the local Mach number
        mach_number = (self.cruise_velocity_horizontal_tail
                       / self.cruise_speed_of_sound)
        # Compute the half chord sweep
        sweep = sweep_to_sweep(0.25,
                               radians(self.horizontal_tail_sweep),
                               0.5,
                               self.horizontal_tail.aspect_ratio,
                               self.horizontal_tail.taper_ratio)
        # Return the derivative of the lift coefficient
        return (2. * pi * self.horizontal_tail.aspect_ratio /
                (2. + sqrt(4. + (self.horizontal_tail.aspect_ratio
                                 * sqrt(1 - mach_number ** 2) / 0.95) ** 2
                           * (1. + (tan(sweep)) ** 2
                              / sqrt(1 -
                                     (self.cruise_velocity_horizontal_tail
                                      / self.cruise_speed_of_sound) ** 2)

                              )
                           )
                 )
                )

    # Determining the required tail area

    @Attribute
    def normalised_centre_of_gravity(self):
        # Define the normalised position of the centre of gravity, related to
        # the leading edge of the mean aerodynamic chord; include 5% margin
        cog = ((self.centre_of_gravity[0] * 1.05
                # Subtract the leading edge position of the mean aerodynamic
                # chord
                - (self.wing_location.x
                   + tan(radians(self.wing_sweep))
                   * self.main_wing.lateral_position_of_mean_aerodynamic_chord
                   - self.main_wing.mean_aerodynamic_chord / 4))
               # Normalise the distance with respect to the mean
               # aerodynamic chord
               / self.main_wing.mean_aerodynamic_chord)
        return cog

    @Attribute
    def horizontal_tail_area_controllability(self):
        # The minimum area required for controllability
        return (((self.normalised_centre_of_gravity +
                  self.moment_coefficient_aerodynamic_centre /
                  self.design_cl - self.aerodynamic_center_wing_and_fuselage)
                 / (self.horizontal_tail_lift_coefficient
                    / self.design_cl
                    * (self.tail_arm /
                       self.main_wing.mean_aerodynamic_chord) * (
                            self.cruise_velocity_horizontal_tail /
                            self.velocity) ** 2)) * self.wing_area)

    @Attribute
    def horizontal_tail_area_stability(self):
        # The minimum area required for stability
        return ((self.normalised_centre_of_gravity - (
                self.aerodynamic_center_wing_and_fuselage - 0.05))
                / (self.lift_coefficient_alpha_horizontal_tail /
                   self.lift_coefficient_alpha_wing_and_fuselage
                   * (1 - self.down_wash) * self.tail_arm /
                   self.main_wing.mean_aerodynamic_chord * (
                           self.cruise_velocity_horizontal_tail
                           / self.velocity) ** 2)
                * self.wing_area)

    @Attribute
    def horizontal_tail_area(self):
        # The horizontal tail area must comply with both the controllability
        # and the stability; thus, the area related to the most stringent
        # requirement is returned
        area = max(self.horizontal_tail_area_controllability,
                   self.horizontal_tail_area_stability)
        # print('HT:', area)
        return area

    # The horizontal tail parts, using the attributes above; the part
    # horizontal_tail is used as a reference; right_horizontal_tail and
    # left_horizontal_tail are visible in the GUI

    @Part(in_tree=False)
    def horizontal_tail(self):
        return LiftingSurface(name='horizontal_tail',
                              number_of_profiles=2,
                              airfoils=['2212', '2212'],
                              is_mirrored=True,
                              span=sqrt(self.horizontal_tail_aspect_ratio
                                        * self.horizontal_tail_area),
                              aspect_ratio=self.horizontal_tail_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.horizontal_tail_sweep,
                              incidence_angle=0,
                              twist=0,
                              dihedral=3,
                              position=self.position.translate(
                                  self.position.Vx,
                                  self.fuselage_length * 0.8,
                                  self.position.Vz,
                                  0.3 * self.cabin_height),
                              color='silver')

    @Part
    def right_horizontal_tail(self):
        return SubtractedSolid(shape_in=self.horizontal_tail.surface,
                               tool=self.fuselage.fuselage_shape,
                               color='silver')

    @Part
    def left_horizontal_tail(self):
        return MirroredShape(shape_in=self.right_horizontal_tail,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='silver')

    # -------------------------------------------------------------------------
    # VERTICAL TAIL
    # -------------------------------------------------------------------------

    @Attribute
    def yaw_moment_propeller(self):
        # Obtain the maximum moment arm for worst case OEI condition
        maximum_arm = self.wing_span / 2
        # Return the yaw moment caused by the most outboard propeller
        return self.thrust_per_propeller * maximum_arm

    @Attribute
    def vertical_tail_arm(self):
        return abs(self.vertical_tail_root_location
                   + tan(radians(self.vertical_tail_sweep))
                   - self.centre_of_gravity[0])

    @Input
    def rudder_chord_ratio(self):
        return 0.3

    @Input
    def rudder_deflection(self):
        return radians(30)

    @Attribute
    def rudder_lift_coefficient_ratio(self):
        # Assuming a value of cl alpha / cl alpha theory of 0.9
        return .75 + 0.1 * self.rudder_chord_ratio

    @Attribute
    def rudder_lift_coefficient(self):
        # Assuming a thickness over chord ratio close to 10%
        return 2 + 0.7 * self.rudder_chord_ratio

    @Attribute
    def vertical_tail_area_controllability(self):
        # A propeller aircraft with fixed pitch propeller is assumed
        yaw_moment_drag = 0.25 * self.yaw_moment_propeller
        # K factor to correct for the sweep of the wing
        k = ((1 - 0.08 * (cos(radians(self.vertical_tail_sweep))) ** 2)
             * (cos(radians(self.vertical_tail_sweep))) ** (3 / 4))
        # Obtain the required surface
        return ((self.yaw_moment_propeller + yaw_moment_drag)
                / (0.5 * self.cruise_density * self.velocity ** 2
                   * self.rudder_deflection
                   * self.rudder_lift_coefficient_ratio
                   * self.rudder_lift_coefficient
                   * k * self.vertical_tail_arm))

    # Required for stability

    @Attribute
    def lift_coefficient_alpha_vertical_tail(self):
        # Compute the local Mach number
        mach_number = (self.cruise_velocity_horizontal_tail
                       / self.cruise_speed_of_sound)
        # Compute the half chord sweep
        sweep = sweep_to_sweep(0.25,
                               radians(self.vertical_tail_sweep),
                               0.5,
                               self.vertical_tail_aspect_ratio,
                               self.vertical_tail_taper_ratio)
        # Return the derivative of the lift coefficient
        return (2. * pi * self.vertical_tail_aspect_ratio /
                (2. + sqrt(4. + (self.vertical_tail_aspect_ratio
                                 * sqrt(1 - mach_number ** 2) / 0.95) ** 2
                           * (1. + (tan(sweep)) ** 2
                              / sqrt(1 - (self.cruise_velocity_horizontal_tail
                                          / self.cruise_speed_of_sound) ** 2)

                              )
                           )
                 )
                )

    @Attribute
    def vertical_tail_area_stability(self):
        return ((self.minimum_side_slip_derivative
                 - self.coefficient_n_beta_fuselage)
                # Cy beta is - Cl alpha, hence - Cy beta = Cl alpha
                / self.lift_coefficient_alpha_vertical_tail
                * self.wing_span
                / self.vertical_tail_arm) * self.wing_area

    @Attribute
    def coefficient_n_beta_fuselage(self):
        factor_k_n = (0.01 *
                      (0.27 * self.centre_of_gravity[0]
                       / self.fuselage_length
                       - 0.168 * log(self.fuselage_length / self.cabin_height)
                       + 0.416)
                      - 0.0005)
        factor_k_R_l = (0.46
                        * log10((self.velocity * self.fuselage_length
                                 / self.kinematic_viscosity_air) / 10. ** 6)
                        + 1.)
        return (-360. / (2. * pi)
                * factor_k_n * factor_k_R_l
                * self.fuselage_length ** 2 * self.cabin_height
                / (self.wing_area * self.wing_span))

    @Input
    def minimum_side_slip_derivative(self):
        return 0.0571

    @Attribute
    def vertical_tail_area(self):
        # Return the area for each single vertical tail, i.e. divide the
        # total required area by the number of tails (which is 2)
        area = max(self.vertical_tail_area_controllability,
                   self.vertical_tail_area_stability) / 2
        # print('VT:', area)
        return area

    # ADJUST THIS THING !!!!!!!!!!

    @Attribute
    def vertical_skid_profile(self):
        return '0012'

    @Attribute
    def vertical_tail_sweep(self):
        return 35

    #     root_position = self.vertical_tail_root_location
    #     tip_position = (self.horizontal_tail.position.x
    #                     + self.lateral_position_of_skids
    #                     * tan(radians(self.horizontal_tail_sweep)))
    #     return degrees(atan((tip_position - root_position)
    #                         / self.vertical_tail_span))

    # @Attribute
    # def vertical_tail_tip_chord(self):
    #     return chord_length(self.horizontal_tail.root_chord,
    #                         self.horizontal_tail.tip_chord,
    #                         self.lateral_position_of_skids
    #                         / (self.horizontal_tail.span / 2))

    @Attribute
    def vertical_tail_span(self):
        return sqrt(self.vertical_tail_aspect_ratio *
                    self.vertical_tail_area)
        # return (self.horizontal_tail.position.z
        #         + self.lateral_position_of_skids
        #         * tan(radians(self.horizontal_tail.dihedral))
        #         - self.vertical_position_of_skids)

    @Attribute
    def vertical_tail_root_chord(self):
        return (2 * (self.vertical_tail_area / self.vertical_tail_span)
                / (1 + self.vertical_tail_taper_ratio))

    @Attribute
    def vertical_tail_taper_ratio(self):
        return 0.6
        # return self.vertical_tail_tip_chord / self.vertical_tail_root_chord

    @Input
    def vertical_tail_aspect_ratio(self):
        return 2
        # return (self.vertical_tail_span /
        #         (self.vertical_tail_root_chord *
        #          (1 + self.vertical_tail_taper_ratio) / 2))

    @Attribute
    def vertical_tail_root_location(self):
        # This provides the location relative to the nose
        vertical = (self.horizontal_tail.position.z
                    - self.vertical_position_of_skids)
        longitudinal = (self.horizontal_tail.position.x
                        + self.lateral_position_of_skids
                        * tan(radians(self.horizontal_tail_sweep))
                        - vertical * tan(radians(self.vertical_tail_sweep)))
        return longitudinal

    # Parts: the vertical_tail is a reference part based on the above
    # attributes; right_vertical_tail and left_vertical_tail are instances
    # visible in the GUI

    @Part(in_tree=True)
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

    # @Part
    # def right_vertical_tail(self):
    #     return SubtractedSolid(shape_in=self.vertical_tail[1].surface,
    #                            tool=[self.right_horizontal_tail,
    #                                  self.landing_skids[1]],
    #                            color='silver')
    #
    # @Part
    # def left_vertical_tail(self):
    #     return MirroredShape(shape_in=self.right_vertical_tail,
    #                          reference_point=self.position,
    #                          vector1=self.position.Vx,
    #                          vector2=self.position.Vz,
    #                          color='silver')

    # -------------------------------------------------------------------------
    # SKIDS
    # -------------------------------------------------------------------------

    # Geometric properties of the skids

    @Attribute
    # The required separation between the VTOL rotors relative to the size
    # of the rotor itself; this impacts the size of the skids
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
                  * 2 * self.prop_separation_factor + 0.1)
        # Provide as output the length of the skid, as well as how the
        # propellers are divided, such that they can be positioned later on
        return [length, distance_between_connections,
                int(props_on_the_front),
                int(usable_number_of_props), int(props_on_the_rear)]

    @Attribute
    def skid_height(self):
        # Make sure that the skid height is smaller than the width if they
        # would be smaller than 0.1 m; otherwise 0.1 m is set as the height
        return min(0.2, 0.9 * self.skid_width)

    @Attribute
    def skid_width(self):
        # Ensure that the width of the skid is 5% larger than the maximum
        # width of the vertical tail
        return max(1.05 * self.vertical_tail_root_chord *
                   (float(self.vertical_skid_profile) / 100), 0.15)

    # Positioning of the skids

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
        # Maintain a margin between the skids and the fuselage such that the
        # propellers do not get closer to the fuselage than 20% of the cabin
        # width
        return 0.7 * self.cabin_width + self.vtol_propeller_radius

    @Attribute
    def vertical_position_of_skids(self):
        # If no wheels are included, the skids are positioned such that
        # there is at least 20% of the cabin height is available as
        # clearance for the propeller if its tip extends to below the
        # fuselage, or for the fuselage itself
        return (min((self.fuselage.nose_height - 0.2) * self.cabin_height -
                    self.r_propeller,
                    - (0.5 + 0.2) * self.cabin_height)
                if self.wheels_included is False
                # If wheels are included, the same clearance to the ground
                # is maintained; hence, the skids can be placed higher
                else min((self.fuselage.nose_height - 0.2)
                         * self.cabin_height - self.r_propeller
                         + self.wheel_radius + self.vertical_rod_length,
                         - (0.5 + 0.2) * self.cabin_height + self.wheel_radius
                         + self.vertical_rod_length)
                )

    @Attribute
    def skid_locations(self):
        # Position the first skid (index 0) on the left side and the second
        # skid (index 1) on the right side
        return [translate(self.position,
                          self.position.Vx,
                          self.longitudinal_position_of_skids,
                          self.position.Vy,
                          - self.lateral_position_of_skids
                          + 2 * self.lateral_position_of_skids * index,
                          self.position.Vz,
                          self.vertical_position_of_skids)
                for index in range(2)]

    # The skids part is used as a reference, while the landing_skids is
    # visible in the GUI: the VTOL rotors are subtracted from the reference
    # part

    @Part(in_tree=False)
    def skids(self):
        return Skid(quantify=2,
                    color=self.secondary_colour,
                    skid_length=self.length_of_skids[0],
                    skid_width=self.skid_width,
                    skid_height=self.skid_height,
                    position=self.skid_locations[child.index])

    @Part
    def landing_skids(self):
        return SubtractedSolid(quantify=2,
                               shape_in=self.skids[child.index].skid,
                               tool=self.arrange_skids[child.index],
                               color='silver')

    # -------------------------------------------------------------------------
    # SKID CONNECTIONS
    # -------------------------------------------------------------------------

    @Attribute
    # The front connection is located such that it is ahead of any doors;
    # however, it is kept at least 20% behind the nose
    def front_connection_location(self):
        return translate(self.position,
                         self.position.Vx,
                         max(self.length_of_fuselage_nose * 3 / 4
                             - self.front_connection_chord * 3 / 4,
                             0.2 * self.length_of_fuselage_nose),
                         self.position.Vz,
                         (2 * self.fuselage.nose_height - 1)
                         * self.cabin_height / 6)

    @Attribute
    # The chord is adjusted such that the thickness is 90% of the height of
    # the skids
    def front_connection_chord(self):
        return min((self.skid_height * 0.8
                    / (float(self.vertical_skid_profile) / 100)),
                   0.5)

    @Attribute
    # Obtain the vertical distance that has to be filled by the connection
    def front_connection_vertical_length(self):
        return (self.vertical_position_of_skids -
                self.front_connection_location.z)

    @Attribute
    # Obtain the horizontal distance that has to be filled by the connection
    def front_connection_horizontal_length(self):
        return (self.lateral_position_of_skids -
                self.front_connection_location.y)

    @Attribute
    # The way the span is defined, it is simply the horizontal length of the
    # connection; the dihedral takes care of the vertical length
    def front_connection_span(self):
        return self.front_connection_horizontal_length

    @Attribute
    # Obtain the angle in degrees between the horizontal plane and the line
    # along the span of the connection
    def front_connection_dihedral(self):
        return degrees(atan(self.front_connection_vertical_length /
                            self.front_connection_horizontal_length))

    # The part right_front_connection_reference is the reference part based
    # on the attributes above and protrudes the fuselage;
    # right_front_connection is a part that is visible in the GUI;
    # left_front_connection is a mirrored instance of right_front_connection.

    @Part(in_tree=False)
    def right_front_connection_reference(self):
        return LiftingSurface(name='front_connections',
                              number_of_profiles=2,
                              airfoils=[self.vertical_skid_profile],
                              is_mirrored=False,
                              span=self.front_connection_span,
                              aspect_ratio=(self.front_connection_span
                                            / self.front_connection_chord),
                              taper_ratio=1,
                              sweep=0,
                              incidence_angle=0,
                              twist=0,
                              dihedral=self.front_connection_dihedral,
                              position=self.front_connection_location,
                              color=self.secondary_colour)

    @Part
    def right_front_connection(self):
        return SubtractedSolid(
            shape_in=self.right_front_connection_reference.surface,
            tool=[self.fuselage.fuselage_shape,
                  self.landing_skids[1]],
            color=self.secondary_colour)

    @Part
    def left_front_connection(self):
        return MirroredShape(shape_in=self.right_front_connection,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # Wheels
    # -------------------------------------------------------------------------

    # Wheel positioning

    @Attribute
    def number_of_wheels(self):
        load_factor_landing = 3.
        max_load_per_tire_kg = 4086
        expected_number_of_wheels = (load_factor_landing
                                     * self.maximum_take_off_weight
                                     / (max_load_per_tire_kg * g))
        number_of_wheels = (2 * ceil(expected_number_of_wheels / 2)
                            if expected_number_of_wheels >= 4 else 4)
        return number_of_wheels

    @Attribute
    def wheel_locations(self):
        # Make sure that there are not more wheels than can fit on the skids
        wheels_per_side = (int(self.number_of_wheels / 2)
                           if (self.number_of_wheels * self.wheel_radius
                               < 0.8 * self.length_of_skids[0])
                           else ceil(0.8 * self.length_of_skids[0] /
                                     (2 * self.wheel_radius)))
        # Provide the locations for the set of wheels on the left side
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
        return 18 / 2 * inch_to_m

    @Attribute
    def wheel_width(self):
        return 5.7 * inch_to_m

    @Attribute
    def rod_radius(self):
        # Make sure that the rod radius is not more than 40% of the skid
        # width (hence the diameter shall be less than 80% of the skid width)
        return min(0.03, self.skid_width * 0.4)

    @Attribute
    def vertical_rod_length(self):
        # Make sure that there is a clearance of 20% of the wheel radius
        # between the wheel and the skid
        return self.wheel_radius * 1.2

    @Attribute
    def horizontal_rod_length(self):
        # Position the wheels 20% outside the skids and the vertical rod
        return self.skid_width * 0.7 + self.rod_radius

    # Wheel parts: right_wheels are mirrored instances of the left_wheels

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

    # Rod parts: the left_wheel_reference_rods contains both the vertical
    # and horizontal rods; left_wheel_horizontal_rods and
    # left_wheel_vertical_rods are isolated instances, where the vertical
    # rods have the skids subtracted from them

    @Part(in_tree=False)
    def left_wheel_reference_rods(self):
        return Rods(quantify=len(self.wheel_locations),
                    wheel_length=self.wheel_width,
                    rod_horizontal_length=self.horizontal_rod_length,
                    rod_vertical_length=self.vertical_rod_length,
                    position=self.wheel_locations[child.index],
                    color='silver',
                    suppress=not self.wheels_included)

    @Part(in_tree=False)
    def left_wheel_horizontal_rods(self):
        return Solid(quantify=len(self.wheel_locations),
                     built_from=self.left_wheel_reference_rods[
                         child.index].rod_horizontal)

    @Part(in_tree=False)
    def left_wheel_vertical_rods(self):
        return SubtractedSolid(quantify=len(self.wheel_locations),
                               shape_in=self.left_wheel_reference_rods[
                                   child.index].rod_vertical,
                               tool=self.skids[0].skid)

    # Rod parts: left_wheel_rods provides the proper combined rods for the
    # left wheels, as visible in the GUI; right_wheel_rods is a set of
    # mirrored instances of left_wheel_rods

    @Part
    def left_wheel_rods(self):
        return Compound(quantify=len(self.wheel_locations),
                        built_from=[
                            self.left_wheel_horizontal_rods[child.index],
                            self.left_wheel_vertical_rods[child.index]],
                        color='silver')

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
    # CRUISE PROPELLERS
    # -------------------------------------------------------------------------

    @Attribute
    def number_of_propellers(self):
        required = ceil(self.total_drag_coefficient * self.velocity ** 2
                        * self.wing_area * self.cruise_density / 2
                        / self.thrust_per_propeller)
        allowed = (self.wing_span - self.cabin_width - 3 *
                   self.propeller_radii) / (2 * self.r_propeller)
        if required > allowed:
            message = 'The propeller radius is too small, yielding too many ' \
                      'propellers to fit on the wing. Please reduce the ' \
                      'propeller radius.'
            generate_warning('Warning: value needs to be changed', message)
            return allowed
        else:
            return required

        # return min(ceil(self.total_drag_coefficient * self.velocity ** 2
        #                 * self.wing_area * self.cruise_density / 2
        #                 / self.thrust_per_propeller),
        #            (self.wing_span - self.cabin_width - 3 *
        #             self.propeller_radii) / (2 * self.r_propeller))

    @Attribute
    def propeller_radii(self):
        radius = self.r_propeller
        # while (2 * floor(self.number_of_propellers / 2) * 2.2 * radius >
        #        (self.wing_span - self.cabin_width - 3 * self.radius)):
        #     radius += 0.05
        return radius

    @Attribute
    def thrust_per_propeller(self):
        return (c_t_cruise * self.cruise_density
                # RPS
                * (mach_number_tip * self.cruise_speed_of_sound
                   / (2 * pi * self.propeller_radii)) ** 2
                # Diameter
                * (2 * self.propeller_radii) ** 4)

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
                    if self.number_of_propellers % 2 == 0
                    else int((self.number_of_propellers - 1) / 2))

        # Position each propeller in y-direction on one wing; make sure they
        # are placed such that the most inboard propeller tip still is 0.5
        # propeller radius away from the fuselage
        y_shift = [self.cabin_width / 2 + 1.5 * self.propeller_radii
                   + ((semi_span
                       - self.cabin_width / 2
                       - 1.5 * self.propeller_radii) / semi_span
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
                                - self.propeller_radii * tan(sweep),
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
        return ([first] + right_wing + left_wing if
                self.number_of_propellers % 2 != 0 else right_wing + left_wing)

    @Part(in_tree=True)
    def cruise_propellers(self):
        return Propeller(name='cruise_propellers',
                         quantify=len(self.propeller_locations),
                         number_of_blades=self.n_blades,
                         blade_radius=self.propeller_radii,
                         nacelle_length=(0.55 * chord_length(
                             self.main_wing.root_chord,
                             self.main_wing.tip_chord,
                             abs(self.propeller_locations[
                                     child.index].y / (self.wing_span / 2)))
                                         + self.propeller_radii
                                         * tan(radians(self.wing_sweep))),
                         nacelle_included=
                         (False if child.index == 0
                                   and len(self.propeller_locations) % 2 == 1
                          else True),
                         aspect_ratio=7,
                         ratio_hub_to_blade_radius=0.15,
                         leading_edge_sweep=0,
                         blade_setting_angle=40,
                         blade_outwash=30,
                         number_of_blade_sections=10,
                         blade_thickness=60,
                         position=rotate90(
                             self.propeller_locations[child.index],
                             - self.position.Vy),
                         color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # VTOL ROTORS
    # -------------------------------------------------------------------------

    @Attribute
    def number_of_vtol_propellers(self):
        n_rotors_computed = (- (self.power_roc + self.power_d_liftingsurface)
                             / self.roc_vertical
                             * (1 / ((self.power_hover + self.power_profile)
                                     / self.roc_vertical - 1500. * pi *
                                     self.r_rotor)
                                )
                             )
        n_rotors_per_side = ceil(n_rotors_computed / 2)
        return n_rotors_per_side * 2

    @Input
    def vtol_propeller_radius(self):
        return self.r_rotor

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
                * (0.97 * mach_number_tip * self.cruise_speed_of_sound) ** 2
                * 0.97 * self.r_rotor)

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
        return (self.c_d_rotor * 1. / 8. * self.cruise_density
                * self.r_rotor / r_over_chord_rotor * self.n_blades
                * (mach_number_tip * self.cruise_speed_of_sound) ** 3
                * self.r_rotor)

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

    # The attribute arrange_skids is used to subtract the propeller cones from
    # the skids

    @Attribute
    def arrange_skids(self):
        first_skid = [self.vtol_propellers[index].hub_cone
                      for index in range(int(self.number_of_vtol_propellers
                                             / 2))]
        second_skid = [self.vtol_propellers[index].hub_cone
                       for index in
                       range(int(self.number_of_vtol_propellers
                                 / 2),
                             self.number_of_vtol_propellers)]
        return [first_skid, second_skid]

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

    @Part
    def vtol_propellers(self):
        return Propeller(name='VTOL_propellers',
                         quantify=self.number_of_vtol_propellers,
                         number_of_blades=self.n_blades,
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
                         number_of_blade_sections=10,
                         blade_thickness=50,
                         position=self.vtol_propeller_locations[child.index],
                         color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # INTERFACE: AVL
    # -------------------------------------------------------------------------

    @Attribute
    def avl_surfaces(self):
        return [self.main_wing.avl_surface,
                self.horizontal_tail.avl_surface]
                # self.vertical_tail[0].avl_surface,
                # self.vertical_tail[1].avl_surface]

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

    # ADJUST THE REFERENCE POINT!

    # -------------------------------------------------------------------------
    # INTERFACE: STEP
    # -------------------------------------------------------------------------

    @Part
    def step_parts(self):
        return STEPWriter(filename=FILENAME,
                          trees=[self])
