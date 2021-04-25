"""
This file contains the complete personal aerial vehicle class. It is
structured in the following way:

- Outside the class:

    - Imports and paths
    - Constants
    - Functions and collections

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

import kbeutils.avl as avl
from parapy.core import *
from parapy.geom import *
from parapy.core.validate import *
from parapy.exchange import STEPWriter

from .avl_configurator import AvlAnalysis
from .functions import *
from .fuselage import Fuselage
from .lifting_surface import LiftingSurface
from .propeller import Propeller
from .skids import Skid
from .wheels import Wheels, Rods

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
OUTPUT_DIR = os.path.join(_module_dir, 'output_files', '')
FILENAME = os.path.join(OUTPUT_DIR, 'pav_assembly.stp', '')

# -----------------------------------------------------------------------------
# CONSTANTS
# -----------------------------------------------------------------------------

# Atmospheric constants
G = 9.80665
GAMMA = 1.4
R = 287

# Conversions between imperial and metric system
LBS_TO_KG = 0.45359237
FT_TO_M = 0.3048
INCH_TO_M = 0.0254

# Thrust coefficient for VTOL rotors: should be between 0.0050 and 0.0060
C_T = 0.0050
# Thrust coefficient for cruise propellers: should be between 0.02 and 0.16
C_T_CRUISE = 0.10
# Solidity factor of the rotor (fixed)
SIGMA_ROTOR = 0.070
# Mach number at the tip of the rotor blade; fixed, higher Mach numbers for
# the rotor tips lead to stall
MACH_NUMBER_TIP = 0.6
# Disk loading in [N/m^2]; should be between 1000 and 7000 N/m^2
DL_MAX = 5000
# Aspect ratio of the rotors: should be between 15 and 20
ASPECT_RATIO_ROTOR: int = 15
# Rotor efficiency factor in hovering condition: should be between 0.6 and 0.8
FIGURE_OF_MERIT = 0.8
# The rotor tip incidence angle is 10 degrees lower than at the root,
# which is good for hover; given in degrees
TWIST_ROTOR = 10
# A factor used to determine the drag of the VTOL rotors: should be between
# 1.1 and 1.2
K_FACTOR_ROTOR_DRAG = 1.15
# The default lift coefficient for cruise is set at 0.5
DESIGN_LIFT_COEFFICIENT = 0.5
# Rate of climb in [m/s]: between 2 and 10 m/s is most common for vtol winged
# aircraft, between 10 and 15 m/s is most common for helicopters
ROC_VERTICAL = 5
# Drag coefficient of a flat plate
C_D_FLAT_PLATE = 1.28
# Number of blades for the VTOL rotors; should be between 2 and 5
N_BLADES_VTOL = 2
# Number of blades for the cruise propellers; should be between 2 and 5
N_BLADES_CRUISE = 4
# Margin in [m] to ensure clearance between the VTOL rotors and the surfaces
# connected to the skid
MARGIN_FOR_TAIL_AND_CONNECTION = 0.2

# -----------------------------------------------------------------------------
# FUNCTIONS AND COLLECTIONS
# -----------------------------------------------------------------------------

# For the internal analysis, only one case is required to run in AVL: the
# lift coefficient is fixed and the angle of attack can vary
cases = [('fixed_cl',
          {'alpha': avl.Parameter(name='alpha',
                                  value=str(DESIGN_LIFT_COEFFICIENT),
                                  setting='CL')})]

# A collection of all valid colours for the GUI; if other colours are
# chosen, a warning is displayed
colours = ['white', 'whitesmoke', 'snow', 'seashell', 'linen', 'oldlace',
           'floralwhite', 'cornsilk', 'ivory', 'beige', 'lightyellow',
           'lightgoldenrodyellow', 'honeydew', 'mintcream', 'azure',
           'lightcyan', 'aliceblue', 'ghostwhite', 'lavender',
           'lavenderblush', 'gainsboro', 'lightgray', 'mistyrose',
           'peachpuff', 'bisque', 'antiquewhite', 'navajowhite',
           'blanchedalmond', 'papayawhip', 'moccasin', 'wheat',
           'lemonchiffon', 'palegoldenrod', 'palegreen', 'aquamarine',
           'paleturquoise', 'powderblue', 'lightblue', 'pink', 'lightpink',
           'silver', 'lightcoral', 'salmon', 'tomato', 'darksalmon',
           'coral', 'lightsalmon', 'sandybrown', 'burlywood', 'tan',
           'khaki', 'greenyellow', 'lightgreen', 'skyblue', 'lightskyblue',
           'lightsteelblue', 'thistle', 'plum', 'violet', 'hotpink',
           'darkgray', 'rosybrown', 'orangered', 'darkorange', 'orange',
           'gold', 'darkkhaki', 'yellow', 'yellowgreen', 'chartreuse',
           'lawngreen', 'darkseagreen', 'mediumaquamarine', 'turquoise',
           'mediumturquoise', 'cornflowerblue', 'mediumslateblue',
           'mediumpurple', 'orchid', 'palevioletred', 'gray', 'indianred',
           'chocolate', 'peru', 'goldenrod', 'limegreen', 'lime',
           'mediumseagreen', 'springgreen', 'mediumspringgreen', 'aqua',
           'cyan', 'cadetblue', 'dodgerblue', 'lightslategray', 'slategray',
           'royalblue', 'slateblue', 'mediumorchid', 'deeppink', 'dimgray',
           'red', 'brown', 'firebrick', 'sienna', 'saddlebrown',
           'darkgoldenrod', 'olivedrab', 'seagreen', 'lightseagreen',
           'darkturquoise', 'deepskyblue', 'steelblue', 'blue',
           'blueviolet', 'darkorchid', 'fuchsia', 'magenta',
           'mediumvioletred', 'crimson', 'black', 'maroon', 'darkred',
           'olive', 'darkolivegreen', 'darkgreen', 'green', 'forestgreen',
           'darkslategray', 'teal', 'darkcyan', 'midnightblue', 'navy',
           'darkblue', 'mediumblue', 'darkslateblue', 'indigo',
           'darkviolet', 'purple', 'darkmagenta']


def thrust_per_propeller(density, speed_of_sound, radius):
    # Computes the thrust of a propeller based on its radius and the
    # atmospheric density and speed of sound
    return (C_T_CRUISE * density
            # RPS
            * (MACH_NUMBER_TIP * speed_of_sound
               / (2 * pi * radius)) ** 2
            # Diameter
            * (2 * radius) ** 4)


# -----------------------------------------------------------------------------
# PAV
# -----------------------------------------------------------------------------


class PAV(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    name = Input()

    # Number of passengers: must be 10 or less
    number_of_passengers = Input(4, validator=And(Positive, LessThan(11)))
    # Range in [km]
    required_range = Input(500)
    # Maximum allowable span in [m]
    maximum_span = Input(12)
    # Quality level can be 1 for economy or 2 for business
    quality_level = Input(1, validator=OneOf([1, 2]))
    # Should wheels be added to allow for driving? True or False
    wheels_included = Input(True)
    # The cruise velocity can be given in [km/hr]
    cruise_velocity = Input(400)
    # The colours that are used for the visualisation
    primary_colours = Input('white')
    secondary_colours = Input('red')
    # To avoid annoyance during design iterations, warnings can be hidden
    # for intermediate steps if hide_warnings is set to True
    hide_warnings = Input(False)

    @Input
    def design_cl(self):
        # A default lift coefficient is provided, but if the user has
        # experience with aerodynamics, they are free to adjust it
        return DESIGN_LIFT_COEFFICIENT

    @Input
    def cruise_altitude_in_feet(self):
        # A default cruise altitude of 10,000 ft is provided, but the user
        # may adjust it if desired. For the validity of the design,
        # it is recommended to keep it well below 20,000 ft.
        return 10e3

    @Input
    def centre_of_gravity(self):
        # This input is used to compute the size of the empennage and
        # locations of the VTOL rotors; it is updated in iterations
        return [.35 * self.fuselage_length, 0, 0.1]

    @Input
    def maximum_take_off_weight(self):
        # This input is used to estimate the size of the wing and other
        # components and is updated in iterations. fr and fv are fractions
        # to correct for changing range or velocity
        fr = 1.5 + (self.range - 100) * 1e3 * 0.0025 / 1e3
        fv = 1.5 + (self.velocity - 100) * 0.0025

        # Return the MTOW in Newtons, based on the payload weight
        return 3.5 * fr * fv * (self.number_of_passengers *
                                (70 + self.quality_level * 15)) * G

    @Input
    def longitudinal_wing_position(self):
        # This input is used to iterate for wing positioning
        return 0.4

    # -------------------------------------------------------------------------
    # INPUT CHECKS
    # -------------------------------------------------------------------------

    @Attribute
    def primary_colour(self):
        # Check if the primary colour is valid; if not, return 'white' as a
        # default option
        if self.primary_colours not in colours:
            message = 'This colour is not available. Please choose a colour ' \
                      'from the palet as provided.'
            generate_warning('Warning: invalid colour', message)
        return (self.primary_colours if self.primary_colours in colours
                else 'white')

    @Attribute
    def secondary_colour(self):
        # Check if the primary colour is valid; if not, return 'red' as a
        # default option
        if self.secondary_colours not in colours:
            message = 'This colour is not available. Please choose a colour ' \
                      'from the palet as provided.'
            generate_warning('Warning: invalid colour', message)
        return (self.secondary_colours if self.secondary_colours in colours
                else 'red')

    @Attribute
    def velocity(self):
        # Check if the velocity is lower than Mach 0.6; if not, the velocity
        # is reduced to Mach 0.6; higher velocities cannot be achieved and
        # it is recommended to set lower velocities for safety reasons
        velocity = self.cruise_velocity / 3.6
        if velocity / self.cruise_speed_of_sound > 0.6:
            message = 'The cruise velocity is set too high. The cruise ' \
                      'velocity will be set to' \
                      '{} km/h.'.format(int(3.6 * 0.6
                                            * self.cruise_speed_of_sound))
            if self.hide_warnings is False:
                generate_warning('Warning: value changed', message)
            return 0.6 * self.cruise_speed_of_sound
        else:
            return velocity

    @Attribute
    def range(self):
        # Check if the range makes sense
        intended_range = self.required_range

        # If the range is between 1000 and 3000 km, it is assumed that the
        # user made a typo and the input is reduced by one order of magnitude
        if 1000 < intended_range < 3000:
            message = 'This range is too high for our PAV. You may ' \
                      'have made a typo; the range will be divided by ' \
                      '10 such that it becomes {:,.1f} km'.format(
                intended_range / 10)
            if self.hide_warnings is False:
                generate_warning('Warning: value changed', message)
            return intended_range / 10

        # If the range is over 500 km but lower than 1000 km, or over 3000 km,
        # it does not make much sense and the range is set back to 500 km
        elif intended_range >= 500:
            message = 'This range is too high for our PAV. ' \
                      'The range will be set to 300 ' \
                      'km'
            if self.hide_warnings is False:
                generate_warning('Warning: value changed', message)
            return 500

        # In other cases, i.e. when the range is below 500 km, this is valid
        # and returned directly
        else:
            return intended_range

    @Attribute
    def wing_span(self):
        # The span as would result from an aspect ratio of 10
        span = sqrt(self.intended_wing_aspect_ratio * self.wing_area)
        # If the span is larger than the maximum span, the maximum span is used
        intermediate_span = (span if span < self.maximum_span else
                             self.maximum_span)
        # Calculate the aspect ratio of the span that is used
        aspect_ratio = intermediate_span ** 2 / self.wing_area
        # If the resulting aspect ratio would be lower than 6, a value of 6
        # is used instead (thus overriding the maximum span constraint)
        if aspect_ratio < 6:
            resulting_span = sqrt(6 * self.wing_area)
            message = 'The maximum span is set too small. ' \
                      'This would yield a very inefficient vehicle. ' \
                      'Therefore, the span is changed to keep an aspect ' \
                      'ratio of 6. This gives a span of {:,.2f} m'.format(
                resulting_span)
            if self.hide_warnings is False:
                generate_warning('Warning: value changed', message)
            return resulting_span

        # If the resulting aspect ratio is larger than 6, the corrected span
        # is returned (adhering to the maximum span constraint)
        else:
            return intermediate_span

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
                ** (-1 - G / (R * -0.0065)))

    @Attribute
    def kinematic_viscosity_air(self):
        # Compute the kinematic viscosity at cruise altitude
        temperature_rankine = self.cruise_temperature * 9. / 5.
        absolute_viscosity = (3.62 * 10 ** -7
                              * (temperature_rankine / 518.7) ** 1.5
                              * (518.7 + 198.72)
                              / (temperature_rankine + 198.72))
        return absolute_viscosity * 47.88 / self.cruise_density

    # Flight velocity related to cruise conditions

    @Attribute
    def cruise_speed_of_sound(self):
        return sqrt(GAMMA * R * self.cruise_temperature)

    @Attribute
    def cruise_mach_number(self):
        return self.velocity / self.cruise_speed_of_sound

    # -------------------------------------------------------------------------
    # FLIGHT PERFORMANCE
    # -------------------------------------------------------------------------

    @Attribute
    def propulsive_efficiency(self):
        # This is the efficiency along the path from the battery exit power
        # to the useful propulsive power
        return 0.9

    @Attribute
    def friction_drag_coefficient(self):
        # The estimated CD0 for the entire vehicle based on references
        return 0.02

    @Attribute
    def analysis(self):
        return AvlAnalysis(aircraft=self,
                           case_settings=cases)

    @Attribute
    def induced_drag_coefficient(self):
        # Obtain the induced drag from the AVL analysis
        analysis = AvlAnalysis(aircraft=self,
                               case_settings=cases)
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
    def battery_energy(self):
        # The energy stored in the battery is given as battery power *
        # battery discharge time
        return self.battery_power * self.battery_discharge_time

    @Attribute
    def battery_mass(self):
        # The battery mass depends on the battery energy and its energy density
        return self.battery_energy / self.battery_energy_density

    # -------------------------------------------------------------------------
    # MASS
    # -------------------------------------------------------------------------

    @Attribute
    def pav_components(self):
        # Identify the components of the PAV

        # For the wing, horizontal tail, vertical tail and front connection,
        # only the right half is provided here, but the mass is taken from
        # both sides; the c.G. will be set at the centre line
        right_wing = self.main_wing.surface
        right_horizontal_tail = self.horizontal_tail.surface
        right_vertical_tail = self.vertical_tail[1].surface
        fuselage = self.fuselage.fuselage_cabin
        propeller = [self.cruise_propellers[index].hub_cone
                     for index in range(len(self.propeller_locations))]
        vtol = [self.vtol_propellers[index].hub_cone
                for index in range(len(self.vtol_propeller_locations))]
        skid = [self.skids[index].skid
                for index in range(len(self.skid_locations))]
        right_front_connection = self.right_front_connection
        wheels = [self.left_wheels[index].wheel
                  for index in range(len(self.wheel_locations))]

        # Return a dictionary with the components (as the payload and
        # battery are not included in the model as parts, they include a
        # value).
        return {'main_wing': right_wing,
                'horizontal_tail': right_horizontal_tail,
                'vertical_tail': right_vertical_tail,
                'wheels': wheels + wheels,
                'fuselage': fuselage,
                'skids': skid,
                'front_connection': right_front_connection,
                'propeller': propeller,
                'vtol': vtol,
                'payload': 0,
                'battery': 0}

    @Attribute
    def center_of_gravity_of_components(self):
        # This attribute computes the c.G. for each component
        dictionary = {}
        for component in self.pav_components:

            # For the parts that are not wheels, skids or propellers,
            # a single point is returned
            if type(self.pav_components[component]) is not list:
                name = component

                # For the battery, it is assumed that the c.G. is positioned
                # along the centre line (by evenly distributing it on top
                # and bottom of the fuselage) and at a quarter of the fuselage
                if name == 'battery':
                    value = [0.25 * self.fuselage_length, 0, 0]

                # The passengers are assumed to have an average c.G. halfway
                # the fuselage, slightly below the centre line as they are
                # seated
                elif name == 'payload':
                    value = [0.5 * self.fuselage_length,
                             -0.2 * self.cabin_height, 0]

                # For the other components, the c.G. is taken directly from
                # that component
                else:
                    value = [self.pav_components[component].cog.x,
                             self.pav_components[component].cog.y,
                             self.pav_components[component].cog.z]

                # For the lifting surfaces, which were only defined on one
                # side, the lateral component of the c.G. is set back to 0,
                # such that the vehicle is symmetric
                if component == 'main_wing' or 'horizontal_tail' \
                        or 'vertical_tail' or 'front_connection':
                    value[1] = 0

                # Add the entries to the dictionary
                library = {name: value}
                dictionary = {**dictionary, **library}

            # For wheels, skids and propellers, the c.G. is directly taken
            # from the components and a list is created
            else:
                values = []
                for index in range(len(self.pav_components[component])):
                    value = [self.pav_components[component][index].cog.x,
                             self.pav_components[component][index].cog.y,
                             self.pav_components[component][index].cog.z]

                    # For the wheels, only the left wheels were defined; to
                    # get the right wheels, the y-coordinate is reversed
                    if (component == 'wheels' and index >=
                            len(self.pav_components[component]) / 2):
                        value[1] = - value[1]
                    values.append(value)

                # Add the names and values from the list to a dictionary
                name = component
                library = {name: values}

                # Add these entries to the dictionary and return the
                # complete set
                dictionary = {**dictionary, **library}
        return dictionary

    @Attribute
    def mass_of_components(self):
        # Approximate the mass of each component based on geometric factors
        return {'main_wing': 40 * self.wing_area,
                'horizontal_tail': 40 * self.horizontal_tail_area,
                'vertical_tail': 40 * self.vertical_tail_area,
                'wheels': 20,
                'fuselage': (2700 * 0.005 * (2 * self.cabin_height
                                             + 2 * self.cabin_width)
                             * self.fuselage_length),
                'skids': 50 * self.length_of_skids * self.skid_width,
                'front_connection': (40 * 2 * (self.front_connection_span
                                               - self.cabin_width / 2)
                                     * self.front_connection_chord),
                'propeller': (5 + N_BLADES_CRUISE * self.propeller_radii[-1]
                              ** 3 / (ASPECT_RATIO_ROTOR ** 2) * 0.12 * 2700),
                'vtol': (5 + N_BLADES_VTOL * self.vtol_propeller_radius
                         ** 3 / (ASPECT_RATIO_ROTOR ** 2) * 0.12 * 2700),
                'battery': self.battery_mass,
                'payload': ((70 + 15 * self.quality_level)
                            * self.number_of_passengers)}

    @Attribute
    def mass(self):
        # Compute the complete mass of the vehicle including battery and
        # payload by summing all the individual components
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
        # Compute the c.G. by summing the mass and length of each component,
        # then dividing by the complete mass
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

    @Attribute
    def expected_maximum_take_off_weight(self):
        # Compute the actual MTOW in Newton; this can be used as input in
        # later iterations
        return self.mass * G

    @Part
    def center_of_gravity_point(self):
        # Show a point indicating the c.G.
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
        return (self.number_of_rows * self.seat_pitch)
        # - self.length_of_fuselage_nose / 4)

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
                        seat_pitch=self.seat_pitch,
                        number_of_rows=self.number_of_rows,
                        door_height=self.cabin_height - 0.4,
                        nose_height=-0.2,
                        tail_height=0.3,
                        pass_down=['hide_warnings', 'primary_colour',
                                   'secondary_colour'])

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
        # For a high wing configuration, the dihedral is set to 1 degree;
        # for low wing configurations, dihedral is set to 3 degrees
        return 1 if self.wing_location.z > 0 else 3

    # Position of the wing

    @Attribute
    def vertical_wing_position(self):
        # The vertical position depends on the root chord, such that the
        # wing does not cover any doors
        return (0.5 * self.cabin_height - 0.10
                if self.main_wing.root_chord < 2
                else 0.5 * self.cabin_height - 0.10 + 0.10
                     * (self.main_wing.root_chord - 2))

    @Attribute
    def wing_location(self):
        # The length ratio is changed as per input for iterations
        length_ratio = self.longitudinal_wing_position
        # The wing is positioned at 90% of the cabin height
        height_ratio = 0.9
        return self.position.translate(self.position.Vx,
                                       length_ratio * self.fuselage_length,
                                       self.position.Vz,
                                       self.vertical_wing_position)

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
                               color='darkgray')

    @Part
    def left_wing(self):
        return MirroredShape(shape_in=self.right_wing,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='darkgray')

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
        return (self.horizontal_tail_longitudinal_position
                - (self.wing_location.x
                   + tan(radians(self.wing_sweep))
                   * self.main_wing.lateral_position_of_mean_aerodynamic_chord)
                )

    # Performance coefficients related to the wing

    @Attribute
    def lift_coefficient_alpha_wing(self):
        # Determine the lift coefficient derivative for the wing only
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
        # Determine the lift coefficient derivative for the wing-fuselage
        # combination
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
        # The horizontal tail lift coefficient depends on its aspect ratio
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
        return area

    @Attribute
    def horizontal_tail_longitudinal_position(self):
        # The horizontal tail is positioned such that it stays in the
        # correct relative location with respect to the vertical tail
        return (self.vertical_tail_root_location
                + tan(radians(self.vertical_tail_sweep))
                * self.vertical_tail_span
                - self.vertical_tail_root_chord / 4
                * self.vertical_tail_taper_ratio)

    @Attribute
    def horizontal_tail_vertical_position(self):
        # The ideal height of the horizontal tail is on top of the vertical
        # tail
        intended_height = (self.vertical_position_of_skids
                           + self.vertical_tail_span
                           - 1.05 * tan(radians(3))
                           * self.lateral_position_of_skids)
        # However, this may yield the horizontal tail ineffective; this
        # depends on the ratio of the relative height and tail arm
        height_relative_to_wing = ((intended_height
                                    - self.vertical_wing_position)
                                   / self.main_wing.mean_aerodynamic_chord)
        relative_tail_arm = (self.tail_arm
                             / self.main_wing.mean_aerodynamic_chord)

        # If the ideal height is low enough, it is used; else,
        # the horizontal tail is placed lower to maintain effectiveness
        return (intended_height
                if height_relative_to_wing / relative_tail_arm < 1 / 7
                else relative_tail_arm / 8
                     * self.main_wing.mean_aerodynamic_chord
                     + self.vertical_wing_position)

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
                                  self.horizontal_tail_longitudinal_position,
                                  self.position.Vz,
                                  self.horizontal_tail_vertical_position),
                              color='silver')

    @Part
    def right_horizontal_tail(self):
        return SubtractedSolid(shape_in=self.horizontal_tail.surface,
                               tool=[self.fuselage.fuselage_shape,
                                     self.right_vertical_tail],
                               color='darkgray')

    @Part
    def left_horizontal_tail(self):
        return MirroredShape(shape_in=self.right_horizontal_tail,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='darkgray')

    # -------------------------------------------------------------------------
    # VERTICAL TAIL
    # -------------------------------------------------------------------------

    @Attribute
    def yaw_moment_propeller(self):
        # Obtain the maximum moment arm for worst case OEI condition
        maximum_arm = self.wing_span / 2
        # Return the yaw moment caused by the most outboard propeller
        return self.thrust_per_propeller[1] * maximum_arm

    @Attribute
    def vertical_tail_arm(self):
        # Determine the distance from the vertical tail to the c.G.; to
        # avoid circular reference, the centre point of the vertical tail is
        # assumed to be halfway the height of the fuselage
        return abs(self.vertical_tail_root_location
                   + tan(radians(self.vertical_tail_sweep))
                   * self.cabin_height / 2
                   - self.centre_of_gravity[0])

    @Attribute
    def rudder_chord_ratio(self):
        # The ratio of the root chord compared to the total chord
        return 0.3

    @Attribute
    def rudder_deflection(self):
        # The maximum deflection of the rudder
        return radians(25)

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
        # Compute the required tail are for stability
        return ((self.minimum_side_slip_derivative
                 - self.coefficient_n_beta_fuselage)
                # Cy beta is - Cl alpha, hence - Cy beta = Cl alpha
                / self.lift_coefficient_alpha_vertical_tail
                * self.wing_span
                / self.vertical_tail_arm) * self.wing_area

    @Attribute
    def coefficient_n_beta_fuselage(self):
        # Compute the side force coefficient derivative for the fuselage,
        # based on semi-empirical formulas
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

    @Attribute
    def minimum_side_slip_derivative(self):
        # This is the minimum side slip derivative that can be used to
        # maintain control
        return 0.0571

    @Attribute
    def vertical_tail_area(self):
        # Return the area for each single vertical tail, i.e. divide the
        # total required area by the number of tails (which is 2)
        area = max(self.vertical_tail_area_controllability,
                   self.vertical_tail_area_stability) / 2
        return area

    @Attribute
    def vertical_skid_profile(self):
        # A symmetric profile with 12 % thickness is used on the vertical tails
        return '0012'

    @Attribute
    def vertical_tail_sweep(self):
        # The quarter chord sweep of the vertical tail is set to 35 degrees
        return 35

    @Attribute
    def vertical_tail_span(self):
        # Compute the span of the vertical tail from aspect ratio and area
        return sqrt(self.vertical_tail_aspect_ratio *
                    self.vertical_tail_area)

    @Attribute
    def vertical_tail_root_chord(self):
        # Compute the root chord of the vertical tail
        return (2 * (self.vertical_tail_area / self.vertical_tail_span)
                / (1 + self.vertical_tail_taper_ratio))

    @Attribute
    def vertical_tail_taper_ratio(self):
        # A taper ratio of 0.6 is used for the vertical tail, based on
        # references
        return 0.6

    @Attribute
    def vertical_tail_aspect_ratio(self):
        # The aspect ratio of the vertical tail is set to 2
        return 2

    @Attribute
    def vertical_tail_root_location(self):
        # This provides the location relative to the nose
        longitudinal = (self.fuselage_length * 0.8
                        + self.lateral_position_of_skids
                        * tan(radians(self.horizontal_tail_sweep))
                        - self.cabin_height
                        * tan(radians(self.vertical_tail_sweep)))
        return longitudinal

    # Parts: the vertical_tail is a reference part based on the above
    # attributes; right_vertical_tail and left_vertical_tail are instances
    # visible in the GUI

    @Part(in_tree=False)
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

    @Part
    def right_vertical_tail(self):
        return SubtractedSolid(shape_in=self.vertical_tail[1].surface,
                               tool=[  # self.right_horizontal_tail,
                                   self.landing_skids[1]],
                               color=self.secondary_colour)

    @Part
    def left_vertical_tail(self):
        return MirroredShape(shape_in=self.right_vertical_tail,
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # SKIDS
    # -------------------------------------------------------------------------

    # Geometric properties of the skids

    @Attribute
    def prop_separation_factor(self):
        # The required separation between the VTOL rotors relative to the size
        # of the rotor itself; this impacts the size of the skids
        return 1.2

    @Attribute
    def length_of_skids(self):
        # The skids are either as long as required by the VTOL rotors or
        # sufficiently long to connect with the front connections and
        # vertical tails
        return max(self.vtol_propeller_locations[-1].x -
                   self.vtol_propeller_locations[0].x +
                   self.vtol_propeller_radius * 2,
                   self.vertical_tail_root_location
                   + self.vertical_tail_root_chord * 1.1
                   - self.length_of_fuselage_nose / 2)

    @Attribute
    def skid_height(self):
        # Make sure that the skid height is smaller than the width if they
        # would be smaller than 0.2 m; otherwise 0.2 m is set as the height
        return min(0.2, 0.9 * self.skid_width)

    @Attribute
    def skid_width(self):
        # Ensure that the width of the skid is 5% larger than the maximum
        # width of the vertical tail
        return max(1.05 * self.vertical_tail_root_chord *
                   (float(self.vertical_skid_profile) / 100), 0.15)

    # Positioning of the skids

    @Attribute
    def longitudinal_position_of_skids(self):
        # Position the skids either based on the VTOL rotors if they are
        # critical, or halfway the fuselage nose cone
        return min(self.vtol_propeller_locations[0].x
                   - self.vtol_propeller_radius,
                   self.length_of_fuselage_nose / 2)

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
        return (min((self.fuselage.nose_height - 0.15) * self.cabin_height -
                    self.propeller_radii[0],
                    - (0.5 + 0.15) * self.cabin_height)
                if self.wheels_included is False
                # If wheels are included, the same clearance to the ground
                # is maintained; hence, the skids can be placed higher
                else min((self.fuselage.nose_height - 0.35)
                         * self.cabin_height - self.propeller_radii[0]
                         + self.wheel_radius + self.vertical_rod_length,
                         - (0.5 + 0.35) * self.cabin_height + self.wheel_radius
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
                    skid_length=self.length_of_skids,
                    skid_width=self.skid_width,
                    skid_height=self.skid_height,
                    position=self.skid_locations[child.index])

    @Part
    def landing_skids(self):
        return SubtractedSolid(quantify=2,
                               shape_in=self.skids[child.index].skid,
                               tool=self.arrange_skids[child.index],
                               color=self.primary_colour)

    # -------------------------------------------------------------------------
    # SKID CONNECTIONS
    # -------------------------------------------------------------------------

    @Attribute
    def front_connection_location(self):
        # The front connection is located such that it is positioned with
        # the half chord at the rear end of the fuselage nose cone
        return translate(self.position,
                         self.position.Vx,
                         (self.length_of_fuselage_nose * 3 / 4
                          + self.front_connection_chord / 4),
                         self.position.Vz,
                         1 / 4 * (self.cabin_height -
                                  self.fuselage.door_height)
                         - self.cabin_height / 2)

    @Attribute
    def front_connection_chord(self):
        # The chord is adjusted such that the thickness is 90% of the height of
        # the skids or 0.5 m if that would be less
        return min((self.skid_height * 0.8
                    / (float(self.vertical_skid_profile) / 100)),
                   0.5)

    @Attribute
    def front_connection_vertical_length(self):
        # Obtain the vertical distance that has to be filled by the connection
        return (self.vertical_position_of_skids -
                self.front_connection_location.z)

    @Attribute
    def front_connection_horizontal_length(self):
        # Obtain the horizontal distance that has to be filled by the
        # connection
        return (self.lateral_position_of_skids -
                self.front_connection_location.y)

    @Attribute
    def front_connection_span(self):
        # The way the span is defined, it is simply the horizontal length of
        # the connection; the dihedral takes care of the vertical length
        return self.front_connection_horizontal_length

    @Attribute
    def front_connection_dihedral(self):
        # Obtain the angle in degrees between the horizontal plane and the line
        # along the span of the connection
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
        # The number of wheels depends on the load that is exerted upon
        # landing; a minimum of 4 wheels is required
        load_factor_landing = 3.
        max_load_per_tire_kg = 4086
        expected_number_of_wheels = (load_factor_landing
                                     * self.maximum_take_off_weight
                                     / (max_load_per_tire_kg * G))
        number_of_wheels = (2 * ceil(expected_number_of_wheels / 2)
                            if expected_number_of_wheels >= 4 else 4)
        return number_of_wheels

    @Attribute
    def wheel_locations(self):
        # Make sure that there are not more wheels than can fit on the skids
        wheels_per_side = (int(self.number_of_wheels / 2)
                           if (self.number_of_wheels * self.wheel_radius
                               < 0.8 * self.length_of_skids)
                           else ceil(0.8 * self.length_of_skids /
                                     (2 * self.wheel_radius)))
        # Provide the locations for the set of wheels on the left side
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
        # The wheel diameter is set to 18 inch
        return 18 / 2 * INCH_TO_M

    @Attribute
    def wheel_width(self):
        # The wheel width is set to 5.7 inch
        return 5.7 * INCH_TO_M

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
                         child.index].rod_horizontal,
                     suppress=not self.wheels_included)

    @Part(in_tree=False)
    def left_wheel_vertical_rods(self):
        return SubtractedSolid(quantify=len(self.wheel_locations),
                               shape_in=self.left_wheel_reference_rods[
                                   child.index].rod_vertical,
                               tool=self.skids[0].skid,
                               suppress=not self.wheels_included)

    # Rod parts: left_wheel_rods provides the proper combined rods for the
    # left wheels, as visible in the GUI; right_wheel_rods is a set of
    # mirrored instances of left_wheel_rods

    @Part
    def left_wheel_rods(self):
        return Compound(quantify=len(self.wheel_locations),
                        built_from=[
                            self.left_wheel_horizontal_rods[child.index],
                            self.left_wheel_vertical_rods[child.index]],
                        color='darkgray',
                        suppress=not self.wheels_included)

    @Part
    def right_wheel_rods(self):
        return MirroredShape(quantify=len(self.wheel_locations),
                             shape_in=self.left_wheel_rods[child.index],
                             reference_point=self.position,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             color='darkgray',
                             suppress=not self.wheels_included)

    # -------------------------------------------------------------------------
    # CRUISE PROPELLERS
    # -------------------------------------------------------------------------

    @Attribute
    def number_of_propellers(self):
        # Determine the number of propellers that is required; this number
        # is even and excludes the propeller on the nose
        required = (2 * ceil((self.total_drag_coefficient * self.velocity ** 2
                              * self.wing_area * self.cruise_density / 2
                              - self.thrust_per_propeller[0])
                             / self.thrust_per_propeller[1] / 2))
        # Compute how many propellers would fit on the wing
        allowed = ((self.wing_span - self.cabin_width
                   - self.propeller_radii[1])
                   / (2 * self.propeller_radii[1]
                      * self.prop_separation_factor))

        # If not all the required propellers fit on the wing, a warning is
        # provided and only those propellers that fit are returned
        if required > allowed:
            message = 'The wing span is too small, yielding too many ' \
                      'propellers to fit on the wing. Please increase the ' \
                      'maximum wingspan or reduce the range and/or velocity.'
            if self.hide_warnings is False:
                generate_warning('Warning: value needs to be changed', message)
            return allowed

        # If all the required propellers fit on the wing, this required
        # number is returned
        else:
            return required

    @Attribute
    def propeller_radii(self):
        # The radii of the propellers depend on the size of the fuselage or
        # wing
        front_prop_radius = 0.4 * self.cabin_width
        wing_prop_radius = min(self.wing_span / 10., 0.7)
        radius = [front_prop_radius, wing_prop_radius]
        return radius

    @Attribute
    def thrust_per_propeller(self):
        # Compute the thrust generated by the propeller placed on the front
        # of the vehicle
        front_prop_thrust = thrust_per_propeller(self.cruise_density,
                                                 self.cruise_speed_of_sound,
                                                 self.propeller_radii[0])
        # Compute the thrust generated by each propeller placed on the wing
        wing_prop_thrust = thrust_per_propeller(self.cruise_density,
                                                self.cruise_speed_of_sound,
                                                self.propeller_radii[1])
        return [front_prop_thrust, wing_prop_thrust]

    @Attribute
    def propeller_locations(self):
        semi_span = self.wing_span / 2
        sweep = radians(self.wing_sweep)
        dihedral = radians(self.wing_dihedral)
        # The first propeller is located at the nose of the plane
        first = translate(self.wing_location,
                          self.position.Vx,
                          - self.wing_location.x,
                          self.position.Vz,
                          - self.wing_location.z
                          + self.fuselage.nose_height * self.cabin_height)

        # Determine the number of propellers on each side
        one_side = int(self.number_of_propellers / 2)

        # Position each propeller in y-direction on one wing; make sure they
        # are placed such that the most inboard propeller tip still is 0.5
        # propeller radius away from the fuselage
        y_shift = [self.cabin_width / 2 + 1.5 * self.propeller_radii[1]
                   + index * self.propeller_radii[1]
                   * 2 * self.prop_separation_factor
                   for index in range(one_side)]

        # Place the propellers just ahead of the leading edge of the right wing
        right_wing = [translate(self.wing_location,
                                self.wing_location.Vx,
                                y_shift[index] * tan(sweep)
                                - 0.3 * chord_length(
                                    self.main_wing.root_chord,
                                    self.main_wing.tip_chord,
                                    y_shift[index] / semi_span)
                                - self.propeller_radii[1] * tan(sweep),
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

        # Return all propeller locations
        return [first] + right_wing + left_wing

    @Part(in_tree=True)
    def cruise_propellers(self):
        return Propeller(name='cruise_propellers',
                         quantify=len(self.propeller_locations),
                         number_of_blades=N_BLADES_CRUISE,
                         blade_radius=self.propeller_radii[0] if
                         child.index == 0 else self.propeller_radii[1],
                         nacelle_length=(0.95 * chord_length(
                             self.main_wing.root_chord,
                             self.main_wing.tip_chord,
                             abs(self.propeller_locations[
                                     child.index].y / (self.wing_span / 2)))
                                         + self.propeller_radii[1]
                                         * tan(radians(self.wing_sweep))),
                         nacelle_included=
                         (False if child.index == 0
                                   and len(self.propeller_locations) % 2 == 1
                          else True),
                         aspect_ratio=7,
                         ratio_hub_to_blade_radius=0.2,
                         leading_edge_sweep=0,
                         blade_setting_angle=40,
                         blade_outwash=30,
                         number_of_blade_sections=10,
                         blade_thickness=60,
                         position=rotate90(
                             self.propeller_locations[child.index],
                             - self.position.Vy),
                         color=self.secondary_colour)

    @Part
    def right_propeller_nacelles(self):
        return SubtractedSolid(quantify=int(len(self.propeller_locations)
                                            - 1),
                               shape_in=
                               self.cruise_propellers[1 + child.index].nacelle,
                               tool=(self.right_wing if child.index <= (len(
                                   self.propeller_locations) - 1) / 2 - 1
                                     else self.left_wing),
                               color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # VTOL ROTORS
    # -------------------------------------------------------------------------

    @Attribute
    def number_of_vtol_propellers(self):
        # The number of rotors that would be needed is the total vertical
        # thrust that is required divided by the vertical thrust per rotor
        n_rotors_computed = (- (self.power_roc + self.power_d_liftingsurface)
                             / ROC_VERTICAL
                             * (1 / ((self.power_hover + self.power_profile)
                                     / ROC_VERTICAL - DL_MAX * pi *
                                     self.vtol_propeller_radius ** 2)
                                )
                             )
        # The number of rotors should be the same on both skids
        n_rotors_per_side = ceil(n_rotors_computed / 2)
        return n_rotors_per_side * 2

    @Attribute
    def vtol_propeller_radius(self):
        # The rotor radius is determined by the length of the fuselage (via
        # the number of rows), but is bound between 0.4 and 0.8 m
        return max(0.4, min(0.8, self.number_of_rows * 0.15))

    @Attribute
    def power_climb(self):
        # The power needed to climb consists of several components
        return (self.power_hover + self.power_roc + self.power_profile +
                self.power_d_liftingsurface)

    @Attribute
    def power_hover(self):
        # The power to hover is the thrust required times the velocity that
        # is induced during climb
        return self.thrust_hover * self.v_induced_climb

    @Attribute
    def thrust_hover(self):
        # The hover thrust depends on the rotor geometry and flight
        # conditions
        return (1. / 6. * N_BLADES_VTOL * 6.6 * C_T / SIGMA_ROTOR
                * self.cruise_density * self.vtol_propeller_radius
                / ASPECT_RATIO_ROTOR
                * (0.97 * MACH_NUMBER_TIP * self.cruise_speed_of_sound) ** 2
                * 0.97 * self.vtol_propeller_radius)

    @Attribute
    def v_induced_climb(self):
        # The induced velocity during climb depends on how fast the vehicle
        # is climbing and on the rotors
        return (ROC_VERTICAL / 2
                + sqrt((ROC_VERTICAL / 2) ** 2 * self.thrust_hover
                       / (2 * self.cruise_density * pi
                          * self.vtol_propeller_radius)))

    @Attribute
    def power_roc(self):
        # Part of the power that is required to obtain the rate of climb is
        # already included in the hover power; the remainder is computed here
        return self.maximum_take_off_weight * ROC_VERTICAL / 2.

    @Attribute
    def power_profile(self):
        # The power required to overcome the drag depends mostly on the
        # rotor size and the tip velocity
        return (self.c_d_rotor * 1. / 8. * self.cruise_density
                * self.vtol_propeller_radius
                / ASPECT_RATIO_ROTOR * N_BLADES_VTOL
                * (MACH_NUMBER_TIP * self.cruise_speed_of_sound) ** 3
                * self.vtol_propeller_radius)

    @Attribute
    def c_d_rotor(self):
        # Return the drag due to the VTOL rotors
        return (8. / (N_BLADES_VTOL * self.vtol_propeller_radius ** 2
                      / ASPECT_RATIO_ROTOR
                      / (pi * self.vtol_propeller_radius ** 2))
                * sqrt(C_T / 2.)
                * (C_T / FIGURE_OF_MERIT - K_FACTOR_ROTOR_DRAG * C_T))

    @Attribute
    def power_d_liftingsurface(self):
        # Return the power that is required to overcome the drag of the wing
        # elements when moved vertically
        return self.vertical_drag_liftingsurfaces * ROC_VERTICAL

    @Attribute
    def vertical_drag_liftingsurfaces(self):
        # Return the drag created by the wing elements if moving vertically;
        # this can be approximated using the drag coefficient of a flat
        # plate; the critical condition is at sea-level
        return (1. / 2. * 1.225 * ROC_VERTICAL ** 2
                * C_D_FLAT_PLATE
                * (self.wing_area + self.horizontal_tail_area))

    @Attribute
    def arrange_skids(self):
        # The attribute arrange_skids is used to subtract the propeller
        # cones from the skids; it returns the hub cones of the VTOL
        # propellers on the separate skids
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
        # Determine how many rotors fit in between the front connection and
        # vertical tail
        vertical_tail_start = (self.vertical_tail_root_location
                               - self.vertical_tail_root_chord / 4)

        front_connection_end = (self.front_connection_location.x
                                + self.front_connection_location.x * 3 / 4)

        distance_in_between = vertical_tail_start - front_connection_end

        # Compute how many rotors would fit in between the front connection
        # and the vertical tail
        rotors_in_between = floor((distance_in_between
                                   - MARGIN_FOR_TAIL_AND_CONNECTION)
                                  / (self.vtol_propeller_radius * 2
                                     * self.prop_separation_factor))

        # Compute how many rotors would be placed either in front of the
        # front connection or behind the vertical tail; this cannot be negative
        rotors_outside = max(self.number_of_vtol_propellers / 2
                             - rotors_in_between, 0)

        # Make sure that the number of rotors in front of the front
        # connection is the same as the number of rotors behind the vertical
        # tail
        rotors_outside_result = (rotors_outside + 1 if rotors_outside % 2 != 0
                                 else rotors_outside)

        # Recompute the number of rotors that shall be placed in between the
        # front connection and the vertical tail; must be positive
        rotors_in_between_result = max(int((self.number_of_vtol_propellers / 2
                                            - rotors_outside_result)), 0)

        # Determine the lateral position of the rotors
        lateral_position_right = ([self.lateral_position_of_skids]
                                  * int(rotors_outside_result
                                        + rotors_in_between_result))
        lateral_position_left = ([- self.lateral_position_of_skids]
                                 * int(rotors_outside_result
                                       + rotors_in_between_result))
        lateral_position = lateral_position_left + lateral_position_right

        # Determine the vertical position of the rotors
        vertical_position = ([self.vertical_position_of_skids]
                             * int((rotors_outside_result
                                    + rotors_in_between_result) * 2))

        # Compute number of rotors in front of the front connection
        rotors_in_front = int(rotors_outside_result / 2)

        # Compute number of rotors behind the vertical tail
        rotors_behind_vt = int(rotors_outside_result / 2)

        new_margin_rotors_in_between = (distance_in_between
                                        - self.vtol_propeller_radius * 2
                                        * self.prop_separation_factor
                                        * rotors_in_between_result)

        positions_in_between = [front_connection_end
                                + new_margin_rotors_in_between / 2
                                + self.vtol_propeller_radius
                                * self.prop_separation_factor
                                + self.vtol_propeller_radius * 2
                                * self.prop_separation_factor * index
                                for index in range(rotors_in_between_result)]

        centre_of_rotors_in_between = (front_connection_end
                                       + new_margin_rotors_in_between / 2
                                       + self.vtol_propeller_radius
                                       * self.prop_separation_factor
                                       * rotors_in_between_result)

        # Relative position of the centre of the rotors to the c.G.;
        # positive if the rotors are placed behind the c.G. and negative if
        # they are placed ahead of it
        relative_position_to_cg = (centre_of_rotors_in_between -
                                   self.centre_of_gravity[0])

        if rotors_in_front > 0:

            # Relevant if the central propellers are placed behind the c.G.
            if relative_position_to_cg > 0:

                # First compute the locations for the rotors behind the
                # vertical tail
                positions_aft = [self.vertical_tail_root_location
                                 + self.vertical_tail_root_chord * 3 / 4
                                 + MARGIN_FOR_TAIL_AND_CONNECTION / 2
                                 + self.vtol_propeller_radius * 2
                                 * self.prop_separation_factor * (index + 0.5)
                                 for index in range(rotors_behind_vt)]
                # Determine the centre of these rotors
                centre_of_rotors_aft = (self.vertical_tail_root_location
                                        + self.vertical_tail_root_chord * 3 / 4
                                        + MARGIN_FOR_TAIL_AND_CONNECTION / 2
                                        + rotors_behind_vt
                                        * self.vtol_propeller_radius
                                        * self.prop_separation_factor)
                # Compute the distance of this centre to the c.G.
                relative_aft_position = (centre_of_rotors_aft -
                                         self.centre_of_gravity[0])
                # Compute the required distance between the front rotors and
                # the c.G. to balance forces
                relative_front_position = ((relative_position_to_cg
                                            * rotors_in_between_result
                                            + relative_aft_position
                                            * rotors_behind_vt)
                                           / rotors_in_front)
                # Determine the centre of these rotors
                center_of_rotors_front = (self.centre_of_gravity[0]
                                          - relative_front_position)
                # Compute the most aft location of the rotors that must be
                # ahead of the front connection
                back_of_rotors_front = (center_of_rotors_front
                                        + rotors_in_front
                                        * self.vtol_propeller_radius
                                        * self.prop_separation_factor)

                # Check if all the front rotors are ahead of the front
                # connection
                if back_of_rotors_front < (self.front_connection_location.x
                                           - self.front_connection_chord
                                           * 1 / 4
                                           - MARGIN_FOR_TAIL_AND_CONNECTION
                                           / 2):

                    # Determine the locations of the front rotors
                    positions_front = [back_of_rotors_front
                                       - self.vtol_propeller_radius * 2
                                       * self.prop_separation_factor
                                       * (rotors_in_front - 0.5 - index)
                                       for index in range(rotors_in_front)]
                    # Combine all longitudinal positions
                    x_positions = (positions_front + positions_in_between
                                   + positions_aft + positions_front
                                   + positions_in_between + positions_aft)

                # If some of the front rotors are placed 'inside' the front
                # connection, they need to be moved forward
                else:

                    # The rear-most rotor is placed just ahead of the front
                    # connection
                    back_of_rotors_front = (self.front_connection_location.x
                                            - self.front_connection_chord
                                            * 1 / 4
                                            - MARGIN_FOR_TAIL_AND_CONNECTION
                                            / 2)
                    # The new locations for the front rotors are computed
                    positions_front = [back_of_rotors_front
                                       - self.vtol_propeller_radius * 2
                                       * self.prop_separation_factor
                                       * (rotors_in_front - 0.5 - index)
                                       for index in range(rotors_in_front)]
                    # Determine the centre of these rotors
                    center_of_rotors_front = (self.front_connection_location.x
                                              - self.front_connection_chord
                                              * 1 / 4
                                              - MARGIN_FOR_TAIL_AND_CONNECTION
                                              / 2 - rotors_in_front
                                              * self.vtol_propeller_radius
                                              * self.prop_separation_factor)
                    # Compute the distance of this centre to the c.G.
                    relative_front_position = (center_of_rotors_front
                                               - self.centre_of_gravity[0])
                    # Compute the required distance between the rear rotors and
                    # the c.G. to balance forces
                    relative_back_position = ((- relative_position_to_cg
                                               * rotors_in_between_result
                                               - relative_front_position
                                               * rotors_in_front)
                                              / rotors_behind_vt)
                    # Determine the centre of these rotors
                    center_of_rotors_back = (self.centre_of_gravity[0]
                                             + relative_back_position)
                    # Determine the most forward position of the rear rotors
                    # to balance the front rotors
                    front_of_rotors_back = (center_of_rotors_back
                                            - (self.vtol_propeller_radius
                                               * self.prop_separation_factor
                                               * rotors_behind_vt))
                    # Determine the locations of the rear rotors
                    positions_aft = [front_of_rotors_back
                                     + self.vtol_propeller_radius * 2
                                     * self.prop_separation_factor
                                     * (index + 0.5)
                                     for index in range(rotors_behind_vt)]
                    # Combine all longitudinal positions
                    x_positions = (positions_front + positions_in_between
                                   + positions_aft + positions_front
                                   + positions_in_between + positions_aft)

                # Return the coordinates of the VTOL rotors
                return [translate(self.position, self.position.Vx,
                                  x_positions[index], self.position.Vy,
                                  lateral_position[index],
                                  self.position.Vz,
                                  vertical_position[index])
                        for index in range(len(x_positions))]

            # Relevant if the central propellers are placed ahead of the c.G.
            else:

                # First compute the locations for the rotors ahead of the
                # front connections
                positions_front = [self.front_connection_location.x
                                   - self.front_connection_chord * 1 / 4
                                   - MARGIN_FOR_TAIL_AND_CONNECTION / 2
                                   - self.vtol_propeller_radius * 2
                                   * self.prop_separation_factor
                                   * (rotors_in_front - 0.5 - index)
                                   for index in range(rotors_in_front)]
                # Determine the centre of these rotors
                center_of_rotors_front = (self.front_connection_location.x
                                          - self.front_connection_chord * 1 / 4
                                          - MARGIN_FOR_TAIL_AND_CONNECTION / 2
                                          - rotors_in_front
                                          * self.vtol_propeller_radius
                                          * self.prop_separation_factor)
                # Compute the distance of this centre to the c.G.
                relative_front_position = (center_of_rotors_front
                                           - self.centre_of_gravity[0])
                # Compute the required distance between the rear rotors and
                # the c.G. to balance forces
                relative_back_position = ((- relative_position_to_cg
                                           * rotors_in_between_result
                                           - relative_front_position
                                           * rotors_in_front)
                                          / rotors_behind_vt)
                # Determine the centre of these rotors
                center_of_rotors_back = (self.centre_of_gravity[0]
                                         + relative_back_position)
                # Determine the most forward position of the rear rotors
                # to balance the front rotors
                front_of_rotors_back = (center_of_rotors_back
                                        - (self.vtol_propeller_radius
                                           * self.prop_separation_factor
                                           * rotors_behind_vt))

                # Check if all the rear rotors are behind the vertical tail
                if front_of_rotors_back > (self.vertical_tail_root_location
                                           + self.vertical_tail_root_chord
                                           * 3 / 4
                                           + MARGIN_FOR_TAIL_AND_CONNECTION
                                           / 2):
                    # Determine the locations of the rear rotors
                    positions_aft = [front_of_rotors_back
                                     + self.vtol_propeller_radius * 2
                                     * self.prop_separation_factor
                                     * (index + 0.5)
                                     for index in range(rotors_behind_vt)]
                    # Combine all longitudinal positions
                    x_positions = (positions_front + positions_in_between
                                   + positions_aft + positions_front
                                   + positions_in_between + positions_aft)

                # If some of the rear rotors are placed 'inside' the
                # vertical tail, they need to be moved aft
                else:

                    # The most forward rotor is placed just behind the
                    # vertical tail
                    front_of_rotors_back = (self.vertical_tail_root_location
                                            + self.vertical_tail_root_chord
                                            * 3 / 4
                                            + MARGIN_FOR_TAIL_AND_CONNECTION
                                            / 2)
                    # The new locations for the rear rotors are computed
                    positions_aft = [front_of_rotors_back
                                     + self.vtol_propeller_radius * 2
                                     * self.prop_separation_factor
                                     * (index + 0.5)
                                     for index in range(rotors_behind_vt)]
                    # Determine the centre of these rotors
                    centre_of_rotors_aft = (self.vertical_tail_root_location
                                            + self.vertical_tail_root_chord
                                            * 3 / 4
                                            + MARGIN_FOR_TAIL_AND_CONNECTION
                                            / 2 + rotors_behind_vt
                                            * self.vtol_propeller_radius
                                            * self.prop_separation_factor)
                    # Compute the distance of this centre to the c.G.
                    relative_aft_position = (centre_of_rotors_aft -
                                             self.centre_of_gravity[0])
                    # Compute the required distance between the front rotors
                    # and the c.G. to balance forces
                    relative_front_position = ((relative_position_to_cg
                                                * rotors_in_between_result
                                                + relative_aft_position
                                                * rotors_behind_vt)
                                               / rotors_in_front)
                    # Determine the centre of these rotors
                    center_of_rotors_front = (self.centre_of_gravity[0]
                                              - relative_front_position)
                    # Determine the most aft position of the front rotors
                    # to balance the rear rotors
                    back_of_rotors_front = (center_of_rotors_front
                                            + (self.vtol_propeller_radius
                                               * self.prop_separation_factor
                                               * rotors_in_front))
                    # Determine the locations of the front rotors
                    positions_front = [back_of_rotors_front
                                       - self.vtol_propeller_radius * 2
                                       * self.prop_separation_factor
                                       * (rotors_in_front - 0.5 - index)
                                       for index in range(rotors_in_front)]
                    # Combine all longitudinal positions
                    x_positions = (positions_front + positions_in_between
                                   + positions_aft + positions_front
                                   + positions_in_between + positions_aft)

                # Return the coordinates of the VTOL rotors
                return [translate(self.position, self.position.Vx,
                                  x_positions[index], self.position.Vy,
                                  lateral_position[index],
                                  self.position.Vz,
                                  vertical_position[index])
                        for index in range(len(x_positions))]

        # If there are no rotors placed outside the central part of the skid
        else:
            # The longitudinal positions are only those of the rotors placed
            # between the front connection and the vertical tail
            x_positions = positions_in_between + positions_in_between

            # Return the coordinates of the VTOL rotors
            return [translate(self.position, self.position.Vx,
                              x_positions[index], self.position.Vy,
                              lateral_position[index],
                              self.position.Vz,
                              vertical_position[index])
                    for index in range(len(x_positions))]

    @Part
    def vtol_propellers(self):
        return Propeller(name='VTOL_propellers',
                         quantify=len(self.vtol_propeller_locations),
                         number_of_blades=N_BLADES_VTOL,
                         blade_radius=self.vtol_propeller_radius,
                         hub_length=1.5 * self.skid_height,
                         nacelle_included=False,
                         aspect_ratio=ASPECT_RATIO_ROTOR,
                         ratio_hub_to_blade_radius=
                         min(0.2, 0.9 * (self.skid_width / 2)
                             / self.vtol_propeller_radius),
                         leading_edge_sweep=0,
                         blade_setting_angle=30,
                         blade_outwash=TWIST_ROTOR,
                         number_of_blade_sections=10,
                         blade_thickness=50,
                         position=self.vtol_propeller_locations[child.index],
                         color=self.secondary_colour)

    # -------------------------------------------------------------------------
    # INTERFACE: AVL
    # -------------------------------------------------------------------------

    @Attribute
    def avl_surfaces(self):
        # The front connections are not taken into account on purpose,
        # as their contributions will be negligible, while the interference
        # may be significant; hence, only the main wing and the empennage is
        # used for the AVL analysis
        return [self.main_wing.avl_surface,
                self.horizontal_tail.avl_surface,
                self.vertical_tail[0].avl_surface,
                self.vertical_tail[1].avl_surface]

    @Attribute
    def avl_reference_point(self):
        # The reference point is taken as the quarter chord point of the
        # MAC, projected on the symmetry plane
        return Point(self.wing_location.x + tan(radians(self.wing_sweep)) *
                     self.main_wing.lateral_position_of_mean_aerodynamic_chord,
                     0, self.vertical_wing_position)

    @Part(in_tree=False)
    def avl_configuration(self):
        return avl.Configuration(name='pav',
                                 reference_area=self.wing_area,
                                 reference_span=self.wing_span,
                                 reference_chord=
                                 self.main_wing.mean_aerodynamic_chord,
                                 reference_point=self.avl_reference_point,
                                 surfaces=self.avl_surfaces,
                                 mach=self.cruise_mach_number)

    # -------------------------------------------------------------------------
    # INTERFACE: STEP
    # -------------------------------------------------------------------------

    @Part(in_tree=False)
    def step_parts(self):
        # This part can be used to write all the relevant parts into a .stp
        # file
        return STEPWriter(filename=FILENAME,
                          trees=[self])
