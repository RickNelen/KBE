from parapy.geom import *
from parapy.core import *

from math import *
import numpy as np


class Fuselage(LoftedShell):
    # The origin of the coordinate system is the intersection of the centre
    # line of the fuselage with the most forward plane of the fuselage;
    # hence, the nose can be positioned below or above the origin.

    # Number of stations at which profiles are generated
    number_of_positions = Input(20)

    # Shape parameters defining the nose and tail cones
    nose_fineness = Input(1)
    tail_fineness = Input(2)

    # Geometric inputs in [m]
    width = Input(2)
    cabin_length = Input(3)
    height = Input(2)

    # Geometric inputs relative to the maximum length:
    widest_point = Input(0.4)
    highest_point = Input(0.5)

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
    def profiles(self):
        return self.profiles_set

    @Part
    def profiles_set(self):
        return InterpolatedCurve(quantify=len(self.x_locations),
                                 is_periodic=False,
                                 points=[Point(x=self.x_locations[
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
                                                  + self.bottom_locations[
                                                      child.index]) / 2),
                                         Point(x=self.x_locations[
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
                                                  + self.bottom_locations[
                                                      child.index]) / 2),
                                         Point(x=self.x_locations[
                                             child.index],
                                               y=0,
                                               z=self.bottom_locations[
                                                   child.index])],
                                 tangents=[Vector(0, 1, 0), Vector(0, 0, 1),
                                           Vector(0, -1, 0), Vector(0, 0, -1),
                                           Vector(0, 1, 0)])
