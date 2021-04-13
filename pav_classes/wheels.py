from math import *
from parapy.geom import *
from parapy.core import *


class Wheels(GeomBase):
    """Wheel geometry, a loft through circles."""

    #: wheel radius
    #: :type: float
    wheel_radius = Input(0.2)
    #: wheel sections
    #: :type: collections.Sequence[float]
    wheel_sections = Input([90, 100, 100, 100, 100, 100, 100, 100, 100, 90])
    #: wheel length
    #: :type: float
    wheel_length = Input(0.08)

    @Attribute
    def section_radius(self):
        """section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        :rtype: collections.Sequence[float]
        """
        return [i * self.wheel_radius / 100. for i in self.wheel_sections]

    @Attribute
    def section_length(self):
        """section length is determined by dividing the wheel
        length by the number of wheel sections.

        :rtype: float
        """
        return self.wheel_length / (len(self.wheel_sections) - 1)

    @Part(in_tree=False)
    def profiles(self):
        return Circle(quantify=len(self.wheel_sections), color="Black",
                      radius=self.section_radius[child.index],
                      # wheel along the Y axis
                      position=rotate90(translate(self.position,
                                                  # circles are in XY plane,
                                                  # thus need rotation
                                                  self.position.Vy,
                                                  child.index
                                                  * self.section_length),
                                        self.position.Vx))

    @Part
    def wheel(self):
        return LoftedSolid(profiles=self.profiles)


class Rods(GeomBase):
    rod_radius = Input(0.02)
    rod_horizontal_length = Input(0.1)
    rod_vertical_length = Input(0.5)
    wheel_length = Input(0.2)

    @Part(in_tree=False)
    def rod_horizontal(self):
        return Cylinder(self.rod_radius, self.rod_horizontal_length,
                        position=rotate90(
                            translate(self.position, self.position.Vy,
                                      self.wheel_length), -self.position.Vx))

    @Part(in_tree=False)
    def rod_vertical(self):
        return Cylinder(self.rod_radius, self.rod_vertical_length,
                        position=rotate90(
                            translate(self.position,
                                      self.position.Vy,
                                      self.wheel_length
                                      + self.rod_horizontal_length
                                      - self.rod_radius),
                            self.position.Vz))

    @Part
    def rods(self):
        return Compound(built_from=[self.rod_horizontal, self.rod_vertical])


if __name__ == '__main__':
    from parapy.gui import display

    obj = Rods(label="wheel")
    display(obj)
