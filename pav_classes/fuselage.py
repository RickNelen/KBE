from parapy.geom import *
from parapy.core import *

from math import *
from .functions import *
import numpy as np


class Fuselage(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # The origin of the coordinate system is the intersection of the centre
    # line of the fuselage with the most forward plane of the fuselage;
    # hence, the nose can be positioned below or above the origin.

    # Name the instance of the fuselage
    name = Input()

    # Number of stations at which profiles are generated
    number_of_positions = Input(20)

    # Shape parameters defining the nose and tail cones
    nose_fineness = Input(1)
    tail_fineness = Input(2)

    # Geometric inputs in [m]
    width = Input(2)
    cabin_height = Input(1.5)
    cabin_length = Input(3)

    # Passenger related
    number_of_rows = Input(2)
    seat_pitch = Input(1)
    door_height = Input(1.6)

    # Geometric inputs relative to the maximum height; note that the sum of
    # half the radius and the height should not exceed [-0.5, 0.5] if you
    # want to have a smooth fuselage

    # Radius of circular sections at nose and tail
    nose_radius_height = Input(0.05)
    tail_radius_height = Input(0.05)
    # Position of centre of nose and tail
    nose_height = Input(0)
    tail_height = Input(0.3)

    # Geometric inputs relative to the maximum width
    nose_radius_width = Input(default=nose_radius_height)
    tail_radius_width = Input(default=tail_radius_height)

    @Input
    def door_width(self):
        # The door width is set to 0.8 for regular seat pitches; if the seat
        # pitch would be smaller, it is adjusted such that each row can fit
        # a door
        return 0.8 if self.seat_pitch >= 1 else self.seat_pitch - 0.1

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

    # Size of the fuselage

    @Attribute
    def height(self):
        # Make sure that the cabin is higher than the doors, even if this is
        # wrongly defined by the input
        if self.cabin_height < self.door_height + 0.1:
            message = 'The doors cannot be higher than the cabin. The ' \
                      'cabin height is increased to {:,.2f} metres in order ' \
                      'to fit the doors; no action is required if this is ' \
                      'okay with the user. Otherwise, please reduce the door' \
                      ' height.'.format(self.door_height + 0.1)
            if self.hide_warnings is False:
                generate_warning('Warning: value changed', message)
            return self.door_height + 0.1
        else:
            return self.cabin_height

    @Attribute
    def nose_length(self):
        # Define the length of the nose
        return self.width * self.nose_fineness

    @Attribute
    def tail_length(self):
        # Define the length of the tail
        return self.width * self.tail_fineness

    @Attribute
    def total_length(self):
        # Define the total length based on the length of the cabin and the
        # length of the nose and tail cones
        return self.cabin_length + self.nose_length + self.tail_length

    # Relative stations along the fuselage

    @Attribute
    def relative_nose_length(self):
        # Define the relative length of the nose
        return self.nose_length / self.total_length

    @Attribute
    def relative_tail_length(self):
        # Define the relative length of the tail
        return self.tail_length / self.total_length

    @Attribute
    def relative_locations(self):
        # Create relative `places' from 0 at the nose to 1 at the tip; note
        # that the central part, i.e. the cabin with constant cross-section,
        # will be created only from the most aft nose profile and the most
        # forward tail profile
        nose = np.linspace(0, self.relative_nose_length,
                           int(self.number_of_positions / 2))
        tail = np.linspace(1 - self.relative_tail_length, 1,
                           int(self.number_of_positions / 2))
        return np.append(nose, tail)

    # Shape of the nose cone

    @Attribute
    def height_nose(self):
        return [((self.nose_radius_height
                  + 1. / self.relative_nose_length
                  * (1 - self.nose_radius_height)
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.height)
                for i in self.relative_locations
                if i <= self.relative_nose_length]

    @Attribute
    def width_nose(self):
        return [((self.nose_radius_width
                  + 1. / self.relative_nose_length
                  * (1 - self.nose_radius_width)
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.width) for i in self.relative_locations
                if i <= self.relative_nose_length]

    @Attribute
    def nose_locations(self):
        return [translate(self.position,
                          # In the longitudinal direction
                          self.position.Vx,
                          i * self.total_length,
                          # In the vertical direction
                          self.position.Vz,
                          ((self.nose_height + self.nose_radius_height / 2.
                            + 1. / self.relative_nose_length
                            * (0.5 - self.nose_height
                               - self.nose_radius_height / 2.)
                            * sqrt(self.relative_nose_length ** 2. -
                                   (i - self.relative_nose_length) ** 2.))
                           * self.height))
                for i in self.relative_locations
                if i <= self.relative_nose_length]

    # Shape of the tail cone

    @Attribute
    def height_tail(self):
        return [((1 - (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * (1 - self.tail_radius_height)
                  * (i - (1 - self.relative_tail_length)) ** 2)) * self.height
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]

    @Attribute
    def width_tail(self):
        return [((1 - (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * (1 - self.tail_radius_width)
                  * (i - (1 - self.relative_tail_length)) ** 2)) * self.width
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]

    @Attribute
    def tail_locations(self):
        return [translate(self.position,
                          # In the longitudinal direction
                          self.position.Vx,
                          i * self.total_length,
                          # In the vertical direction
                          self.position.Vz,
                          ((0.5 -
                            (1 / (1 - (1 - self.relative_tail_length))) ** 2
                            * (0.5 - self.tail_height
                               - self.tail_radius_height / 2)
                            * (i - (1 - self.relative_tail_length)) ** 2))
                          * self.height)
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]

    # Doors

    @Attribute
    def right_doors(self):
        return (self.doors[index] if index % 2 == 1 else None for
                index in range(len(self.door_profile)))

    @Attribute
    def left_doors(self):
        return (self.doors[index] if index % 2 == 0 else None for
                index in range(len(self.door_profile)))

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    # nose_reference and tail_reference create squares with the proper
    # height and width for the nose cone and tail cone respectively

    @Part(in_tree=False)
    def nose_reference(self):
        return Rectangle(quantify=len(self.height_nose),
                         width=self.height_nose[child.index],
                         length=self.width_nose[child.index],
                         position=rotate90(translate(
                             self.nose_locations[child.index],
                             self.position.Vz,
                             - self.height_nose[child.index] / 2),
                             self.position.Vy))

    @Part(in_tree=False)
    def tail_reference(self):
        return Rectangle(quantify=len(self.height_tail),
                         width=self.height_tail[child.index],
                         length=self.width_tail[child.index],
                         position=rotate90(translate(
                             self.tail_locations[child.index],
                             self.position.Vz,
                             - self.height_tail[child.index] / 2),
                             self.position.Vy))

    # nose_profiles and tail_profiles create the filleted shapes (i.e. with
    # rounded corners) for the nose and tail cones respectively; the fillet
    # is 1/3 of the smallest line, which can be either the width or the height

    @Part(in_tree=False)
    def nose_profiles(self):
        return FilletedWire(quantify=len(self.nose_reference),
                            built_from=self.nose_reference[child.index],
                            radius=min(self.width_nose[child.index],
                                       self.height_nose[child.index]) / 3)

    @Part(in_tree=False)
    def tail_profiles(self):
        return FilletedWire(quantify=len(self.tail_reference),
                            built_from=self.tail_reference[child.index],
                            radius=min(self.width_tail[child.index],
                                       self.height_tail[child.index]) / 3)

    @Part(in_tree=False)
    def fuselage_nose_cone(self):
        return LoftedSolid(profiles=self.nose_profiles)

    @Part(in_tree=False)
    def fuselage_cabin(self):
        return LoftedSolid(profiles=[self.nose_profiles[-1],
                                     self.tail_profiles[0]])

    @Part(in_tree=False)
    def fuselage_tail_cone(self):
        return LoftedSolid(profiles=self.tail_profiles)

    @Part
    def fuselage_shape(self):
        return Compound(built_from=[self.fuselage_nose_cone,
                                    self.fuselage_cabin,
                                    self.fuselage_tail_cone])

    # Door parts: the door_profile provides the shape of the door,
    # while doors provides the projected shape on the fuselage

    @Part
    def door_profile(self):
        return Rectangle(quantify=self.number_of_rows,
                         width=self.door_width,
                         length=self.door_height,
                         position=translate(rotate90(self.position,
                                                     self.position.Vx),
                                            self.position.Vx,
                                            self.nose_length +
                                            child.index * self.seat_pitch,
                                            # local y-axis is the global z-axis
                                            self.position.Vy,
                                            -self.door_height / 50,
                                            # local z-axis is the global
                                            # negative y-axis
                                            self.position.Vz, 0))

    @Part
    def doors(self):
        return ProjectedCurve(quantify=len(self.door_profile),
                              source=self.door_profile[child.index],
                              target=self.fuselage_shape,
                              direction=self.position.Vy,
                              color='white')
