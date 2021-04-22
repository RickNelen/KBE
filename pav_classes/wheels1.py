#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016 ParaPy Holding B.V.
#
# This file is subject to the terms and conditions defined in
# the license agreement that you have received with this source code
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR
# PURPOSE.

# from parapy.geom import *
# from parapy.core import *
#
import os.path

from kbeutils.geom import *
import kbeutils.avl as avl
from parapy.core import *
from parapy.geom import *
from .pav import PAV
from math import *


class Connections(GeomBase):

    @Attribute
    def front_connection_chord(self):
        return (self.PAV.skid_height * 0.9
                / (float(self.pav.vertical_skid_profile) / 100))

    @Attribute
    def front_connection_location(self):
        return translate(self.position,
                         self.position.Vx,
                         self.pav.length_of_fuselage_nose
                         - self.pav.front_connection_chord / 4,
                         self.position.Vz,
                         (2 * self.pav.fuselage.nose_height - 1)
                         * self.pav.cabin_height / 6)

    @Attribute
    def front_connection_vertical_length(self):
        return (self.pav.vertical_position_of_skids -
                self.pav.front_connection_location.z)

    @Attribute
    def front_connection_horizontal_length(self):
        return (self.pav.lateral_position_of_skids -
                self.pav.front_connection_location.y)

    @Attribute
    def front_connection_span(self):
        return self.pav.front_connection_horizontal_length

    @Attribute
    def front_connection_dihedral(self):
        return degrees(atan(self.pav.front_connection_vertical_length /
                            self.pav.front_connection_horizontal_length))

    # distance_in_between = vertical_tail_start - front_connection_end
    #
    # # Compute how many rotors would fit in between the front connection
    # # and the vertical tail
    # rotors_in_between = floor(distance_in_between
    #                           # - margin_for_tail_and_connection)
    #                           / (self.vtol_propeller_radius * 2
    #                              * self.prop_separation_factor))
    #
    # # Compute how many rotors would be placed either in front of the
    # # front connection or behind the vertical tail
    # rotors_outside = self.number_of_vtol_propellers / 2 - rotors_in_between
    #
    # # Make sure that the number of rotors in front of the front
    # # connection is the same as the number of rotors behind the vertical
    # # tail
    # rotors_outside_result = (rotors_outside + 1 if rotors_outside % 2 != 0
    #                          else rotors_outside)
    #
    # # Recompute the number of rotors that shall be placed in between the
    # # front connection and the vertical tail
    # rotors_in_between_result = int((self.number_of_vtol_propellers / 2
    #                                 - rotors_outside_result))

#
# class Wheels(LoftedSolid):
#
#     """Wheel geometry, a loft through circles."""
#
#     #: wheel radius
#     #: :type: float
#     wheel_radius = Input(0.2)
#     #: wheel sections
#     #: :type: collections.Sequence[float]
#     wheel_sections = Input([90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 90])
#     #: wheel length
#     #: :type: float
#     wheel_length = Input(0.08)
#
#     @Attribute
#     def section_radius(self):
#         """section radius multiplied by the radius distribution
#         through the length. Note that the numbers are percentages.
#
#         :rtype: collections.Sequence[float]
#         """
#         return [i * self.wheel_radius / 100. for i in self.wheel_sections]
#
#     @Attribute
#     def section_length(self):
#         """section length is determined by dividing the wheel
#         length by the number of wheel sections.
#
#         :rtype: float
#         """
#         return self.wheel_length / (len(self.wheel_sections) - 1)
#
#     @Attribute  # used by the superclass LoftedSolid. It could be removed if the @Part profile_set /
#     # would be renamed "profiles" and LoftedSolid specified as superclass for Wheel
#     def profiles(self):
#         return self.profile_set  # collect the elements of the sequence profile_set
#
#     @Part
#     def profile_set(self):
#         return Circle(quantify=len(self.wheel_sections), color="Black",
#                       radius=self.section_radius[child.index],
#                       # wheel along the Y axis
#                       position=translate(self.position,  # circles are in XY plane, thus need rotation
#                                          Vector(0, 0, 1),
#                                          child.index * self.section_length))
#
#
#
#
# if __name__ == '__main__':
#     from parapy.gui import display
#     obj = Wheels(label="wheel", mesh_deflection=0.0001)
#     display(obj)
