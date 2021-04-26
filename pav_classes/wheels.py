"""
This file contains two classes: one for the wheels, and one for the rods
connecting the wheels with the skids
"""

# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from parapy.geom import *
from parapy.core import *


# -----------------------------------------------------------------------------
# CLASS WHEELS
# -----------------------------------------------------------------------------


class Wheels(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # Get wheel radius
    wheel_radius = Input(0.33)

    # Get relative radii for sections along the width of the wheel
    wheel_sections = Input([90, 100, 100, 100, 100, 100, 100, 100, 100, 90])

    # Get the width of the wheel
    wheel_length = Input(0.13)

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

    @Attribute
    def section_radius(self):
        # Return the actual section radius (instead of a percentage of the
        # radius) for each section along the width of the wheel
        return [i * self.wheel_radius / 100. for i in self.wheel_sections]

    @Attribute
    def section_length(self):
        # Determine the width of each section such that all sections
        # together fit inside the wheel
        return self.wheel_length / (len(self.wheel_sections) - 1)

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    @Part(in_tree=False)
    def profiles(self):
        # Create a set of profiles from the sections (a set of circles with
        # varying radii at some position along the width of the wheel)
        return Circle(quantify=len(self.wheel_sections), color="Black",
                      radius=self.section_radius[child.index],
                      # Rotate the wheel such that the rotational axis of
                      # the wheel is placed along the Y axis
                      position=rotate90(translate(self.position,
                                                  self.position.Vy,
                                                  child.index
                                                  * self.section_length),
                                        self.position.Vx))

    @Part
    def wheel(self):
        # Create the wheel shape, based on the profiles
        return LoftedSolid(profiles=self.profiles)


# -----------------------------------------------------------------------------
# CLASS RODS
# -----------------------------------------------------------------------------


class Rods(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # All the required parameters for the rods are provided as inputs;
    # hence, no attributes are needed

    # Get the radius of both rods, the length of the horizontal rod and the
    # length of the vertical rod
    rod_radius = Input(0.02)
    rod_horizontal_length = Input(0.1)
    rod_vertical_length = Input(0.5)
    # Get the width of the wheel
    wheel_length = Input(0.2)

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    @Part(in_tree=False)
    def rod_horizontal(self):
        # Create the horizontal rod based on extruding a circle with the rod
        # radius along the length of the horizontal rod
        return Cylinder(self.rod_radius, self.rod_horizontal_length,
                        position=rotate90(translate(self.position,
                                                    self.position.Vy,
                                                    self.wheel_length),
                                          # Rotate 90 degrees to make the
                                          # rod horizontal
                                          -self.position.Vx))

    @Part(in_tree=False)
    def rod_vertical(self):
        # Create the vertical rod based on extruding a circle with the rod
        # radius along the length of the vertical rod
        return Cylinder(self.rod_radius, self.rod_vertical_length,
                        position=translate(self.position,
                                           self.position.Vy,
                                           self.wheel_length
                                           + self.rod_horizontal_length
                                           - self.rod_radius))

    @Part
    def rods(self):
        # Combine both rods into a single part
        return Compound(built_from=[self.rod_horizontal, self.rod_vertical])
