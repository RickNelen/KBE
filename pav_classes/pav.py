from math import *
from parapy.geom import *
from parapy.core import *

from .fuselage import Fuselage
from .lifting_surface import LiftingSurface
from .airfoil import Airfoil
from .propeller import Propeller


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
    @Attribute
    def seat_pitch(self):
        return 1.5 if self.quality_level == 2 else 1

    @Attribute
    def seat_width(self):
        return 0.9 if self.quality_level == 2 else 0.6

    @Attribute
    def cabin_length(self):
        return (ceil(self.number_of_passengers / self.number_of_seats_abreast)
                * self.seat_pitch)

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
        return 250

    # -------------------------------------------------
    # TO DO: implement correct formula!!!!
    # -------------------------------------------------

    @Attribute
    def cruise_density(self):
        return 1.225 - 0.0001 * self.cruise_altitude

    # -------------------------------------------------
    # TO DO: implement correct formula!!!!
    # -------------------------------------------------

    @Attribute
    def cruise_speed_of_sound(self):
        return sqrt(1.4 * 287 * self.cruise_temperature)

    @Attribute
    def cruise_mach_number(self):
        mach = self.intended_velocity / self.cruise_speed_of_sound
        return mach if mach < 0.6 else 0.6

    @Attribute
    def maximum_take_off_weight(self):
        # MTOW in Newtons
        return (2000 + self.number_of_passengers * 70) * 9.81

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
    def wing_location(self):
        length_ratio = 0.4
        height_ratio = 0.8
        return self.position.translate('x', length_ratio *
                                       self.fuselage_length,
                                       'z', (height_ratio - 0.5)
                                       * self.cabin_height)

    @Part
    def main_wing(self):
        return LiftingSurface(name='main_wing',
                              number_of_profiles=4,
                              airfoils=['34018', '33515', '43012', '43010'],
                              is_mirrored=True,
                              span=self.wing_span,
                              aspect_ratio=self.wing_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep,
                              incidence_angle=0,
                              twist=-3,
                              dihedral=2,
                              position=self.wing_location,
                              color=self.secondary_colour)

    @Part
    def horizontal_tail(self):
        return LiftingSurface(name='horizontal_tail',
                              number_of_profiles=2,
                              airfoils=['0018', '0012'],
                              is_mirrored=True,
                              span=self.wing_span/3,
                              aspect_ratio=self.wing_aspect_ratio,
                              taper_ratio=0.4,
                              sweep=self.wing_sweep+5,
                              incidence_angle=0,
                              twist=0,
                              dihedral=3,
                              position=self.position.translate('x',
                                                               self.fuselage_length*0.8,
                                                               'z',
                                                               -0.4*self.cabin_height),
                              color=self.secondary_colour)

    @Part
    def fuselage(self):
        return Fuselage(name='fuselage',
                        number_of_positions=50,
                        nose_fineness=(self.length_of_fuselage_nose
                                       / self.cabin_width),
                        tail_fineness=(self.length_of_fuselage_tail
                                       / self.cabin_width),
                        width=self.cabin_width,
                        height=self.cabin_height,
                        cabin_length=self.cabin_length,
                        nose_radius_height=0.1,
                        tail_radius_height=0.05,
                        nose_height=-0.2,
                        tail_height=0.4,
                        color=self.primary_colour)

