from parapy.geom import *
from parapy.core import *

from math import *
import numpy as np


class Fuselage(LoftedSolid):
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
    height = Input(1.5)
    cabin_length = Input(3)

    # Passenger related
    number_of_rows = Input(2)
    seat_pitch = Input(1)

    door_height = Input(1.6)
    # door_width = Input(0.8)

    # Geometric inputs relative to the maximum height; note that the sum of
    # half the radius and the height should not exceed [-0.5, 0.5] if you
    # want to have a smooth fuselage
    # Radius of circular sections at nose and tail
    nose_radius_height = Input(0.2)
    tail_radius_height = Input(0.1)
    # Position of centre of nose and tail
    nose_height = Input(0)
    tail_height = Input(0.3)

    # Geometric inputs relative to the maximum width
    nose_radius_width = Input(default=nose_radius_height)
    tail_radius_width = Input(default=tail_radius_height)

    @Input
    def door_width(self):
        return 0.8 if self.seat_pitch >= 1 else self.seat_pitch - 0.1

    # Define the actual length of the nose
    @Attribute
    def nose_length(self):
        return self.width * self.nose_fineness

    # Define the actual length of the tail
    @Attribute
    def tail_length(self):
        return self.width * self.tail_fineness

    # Define the total length based on the length of the cabin and the
    # length of the nose and tail cones
    @Attribute
    def total_length(self):
        return self.cabin_length + self.nose_length + self.tail_length

    # Define the relative length of the nose
    @Attribute
    def relative_nose_length(self):
        return self.nose_length / self.total_length

    # Define the relative length of the tail
    @Attribute
    def relative_tail_length(self):
        return self.tail_length / self.total_length

    # Create relative `planes' from 0 at the nose to 1 at the tip
    @Attribute
    def relative_locations(self):
        return np.linspace(0, 1, self.number_of_positions)

    # Define the actual sized x coordinates of the relative planes
    @Attribute
    def x_locations(self):
        return self.total_length * self.relative_locations

    @Attribute
    def top_locations(self):
        # Define the shape of the nose cone along the top at the centre
        nose = [((self.nose_height + self.nose_radius_height / 2.
                  + 1. / self.relative_nose_length
                  * (0.5 - self.nose_height - self.nose_radius_height / 2.)
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.height) for i in self.relative_locations
                if i <= self.relative_nose_length]
        # Define the shape of the tail cone along the top at the centre
        tail = [((0.5 - (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * (0.5 - self.tail_height - self.tail_radius_height / 2)
                  * (i - (1 - self.relative_tail_length)) ** 2)) * self.height
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]
        # Define the shape of the central part along the top at the centre
        cabin = [self.height / 2 for i in self.relative_locations if
                 self.relative_nose_length < i < (
                         1 - self.relative_tail_length)]
        # Combine the three sections into one list
        return nose + cabin + tail

    @Attribute
    def bottom_locations(self):
        # Define the shape of the nose cone along the bottom at the centre
        nose = [((self.nose_height - self.nose_radius_height / 2.
                  - 1. / self.relative_nose_length
                  * (0.5 + self.nose_height - self.nose_radius_height / 2.)
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.height) for i in self.relative_locations
                if i <= self.relative_nose_length]
        # Define the shape of the tail cone along the bottom at the centre
        tail = [((-0.5 + (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * (0.5 + self.tail_height - self.tail_radius_height / 2)
                  * (i - (1 - self.relative_tail_length)) ** 2)) * self.height
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]
        # Define the shape of the central part along the bottom at the centre
        cabin = [-self.height / 2 for i in self.relative_locations if
                 self.relative_nose_length < i < (
                         1 - self.relative_tail_length)]
        # Combine the three sections into one list
        return nose + cabin + tail

    @Attribute
    def side_locations(self):
        # Define the shape of the nose cone along the top at the centre
        nose = [((self.nose_radius_width / 2.
                  + 1. / self.relative_nose_length
                  * (0.5 - self.nose_radius_width / 2.)
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.width) for i in self.relative_locations
                if i <= self.relative_nose_length]
        # Define the shape of the tail cone along the top at the centre
        tail = [((0.5 - (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * (0.5 - self.tail_radius_width / 2)
                  * (i - (1 - self.relative_tail_length)) ** 2)) * self.width
                for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]
        # Define the shape of the central part along the top at the centre
        cabin = [self.width / 2 for i in self.relative_locations if
                 self.relative_nose_length < i < (
                         1 - self.relative_tail_length)]
        # Combine the three sections into one list
        return nose + cabin + tail

    @Attribute
    def locations(self):
        # Define the shape of the nose cone along the top at the centre
        nose = [((1. / self.relative_nose_length * 0.5
                  * sqrt(self.relative_nose_length ** 2. -
                         (i - self.relative_nose_length) ** 2.))
                 * self.width) for i in self.relative_locations
                if i <= self.relative_nose_length]
        # Define the shape of the tail cone along the top at the centre
        tail = [((0.5 - (1 / (1 - (1 - self.relative_tail_length))) ** 2
                  * 0.5 * (i - (1 - self.relative_tail_length)) ** 2))
                * self.width for i in self.relative_locations if
                i >= (1 - self.relative_tail_length)]
        # Define the shape of the central part along the top at the centre
        cabin = [self.width / 2 for i in self.relative_locations if
                 self.relative_nose_length < i < (
                         1 - self.relative_tail_length)]
        # Combine the three sections into one list
        x_locations = self.x_locations
        y_locations = [0] * len(self.x_locations)
        z_locations = nose + cabin + tail
        return list(zip(x_locations, y_locations, z_locations))

    @Part
    def door_profile(self):
        return Rectangle(quantify=self.number_of_rows,
                         width=self.door_width,
                         length=self.door_height,
                         position=translate(rotate90(self.position, 'x'),
                                            'x', self.nose_length +
                                            child.index * self.seat_pitch,
                                            # local y-axis is the global z-axis
                                            'y', -self.door_height/50,
                                            # local z-axis is the global
                                            # negative y-axis
                                            'z', 0))

    @Part
    def doors(self):
        return ProjectedCurve(quantify=len(self.door_profile),
                              source=self.door_profile[child.index],
                              target=self.fuselage_surface,
                              direction=self.position.Vy,
                              color='white')

    @Attribute
    def right_doors(self):
        return (self.doors.wires[index] if index % 2 == 1 else None for
                index in range(len(self.door_profile)))

    @Attribute
    def left_doors(self):
        return (self.doors.wires[index] if index % 2 == 0 else None for
                index in range(len(self.door_profile)))

    @Attribute
    def profiles(self):
        return self.profiles_set

    @Attribute
    def z_locations(self):
        return [0] * len(self.x_locations)

    @Part
    def fuselage_curve(self):
        return FittedCurve(points=self.locations)

    @Part
    def fuselage_surface(self):
        return RevolvedSurface(basis_curve=self.fuselage_curve,
                               mesh_deflection=0.0001,
                               direction=Vector(1, 0, 0))

    @Part
    def profiles_set(self):
        return InterpolatedCurve(quantify=len(self.x_locations),
                                 is_periodic=False,
                                 points=[Point(x=self.x_locations[
                                     child.index],
                                               y=0,
                                               z=self.top_locations[
                                                   child.index]),
                                         Point(x=self.x_locations[
                                             child.index],
                                               y=-self.side_locations[
                                                   child.index],
                                               z=(self.top_locations[
                                                      child.index]
                                                  - self.bottom_locations[
                                                      child.index]) * 1 / 3
                                                 + self.bottom_locations[
                                                     child.index]),
                                         Point(x=self.x_locations[
                                             child.index],
                                               y=0,
                                               z=self.bottom_locations[
                                                   child.index]),
                                         Point(x=self.x_locations[
                                             child.index],
                                               y=self.side_locations[
                                                   child.index],
                                               z=(self.top_locations[
                                                      child.index]
                                                  - self.bottom_locations[
                                                      child.index]) * 1 / 3
                                                 + self.bottom_locations[
                                                     child.index]),
                                         Point(x=self.x_locations[
                                             child.index],
                                               y=0,
                                               z=self.top_locations[
                                                   child.index])],
                                 tangents=[Vector(0, -1, 0), Vector(0, 0, -1),
                                           Vector(0, 1, 0), Vector(0, 0, 1),
                                           Vector(0, -1, 0)])
