from math import *
from parapy.geom import *
from parapy.core import *


class Wheels(GeomBase):

    """Wheel geometry, a loft through circles."""

    #: wheel radius
    #: :type: float
    wheel_radiusw = Input(0.2)
    #: wheel sections
    #: :type: collections.Sequence[float]
    wheel_sectionsw = Input([90, 100, 100, 100, 100, 100, 100, 100, 100, 100, 90])
    #: wheel length
    #: :type: float
    wheel_lengthw = Input(0.08)

    rod_radius = Input(0.02)
    rodh_length = Input(0.1)
    rodv_length = Input(0.5)

    @Attribute
    def section_radiusw(self):
        """section radius multiplied by the radius distribution
        through the length. Note that the numbers are percentages.

        :rtype: collections.Sequence[float]
        """
        return [i * self.wheel_radiusw / 100. for i in self.wheel_sectionsw]

    @Attribute
    def section_lengthw(self):
        """section length is determined by dividing the wheel
        length by the number of wheel sections.

        :rtype: float
        """
        return self.wheel_lengthw / (len(self.wheel_sectionsw) - 1)


    @Attribute  # used by the superclass LoftedSolid. It could be removed if the @Part profile_set /
    # would be renamed "profiles" and LoftedSolid specified as superclass for Wheel
    def profiles(self):
        return self.profile_set  # collect the elements of the sequence profile_set

    @Part
    def profile_set(self):
        return Circle(quantify=len(self.wheel_sectionsw), color="Black",
                      radius=self.section_radiusw[child.index],
                      # wheel along the Y axis
                      position=rotate90(translate(self.position,  # circles are in XY plane, thus need rotation
                                         Vector(0, 1, 0),
                                         child.index * self.section_lengthw),self.position.Vx))

    @Part
    def wheel(self):
        return LoftedSolid(self.profiles)



    @Part
    def rodh(self):
        return Cylinder(self.rod_radius, self.rodh_length,
                        position = rotate90(translate(self.position, self.position.Vy,self.wheel_lengthw), -self.position.Vx))

    @Part
    def rodv(self):
        return Cylinder(self.rod_radius, self.rodv_length,
                        position=rotate90(translate(self.position, self.position.Vy, self.wheel_lengthw + self.rodh_length - self.rod_radius),self.position.Vz))




if __name__ == '__main__':
    from parapy.gui import display
    obj = Wheels(label="wheel")
    display(obj)
