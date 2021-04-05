import os.path

from kbeutils.geom import *
import kbeutils.avl as avl
from parapy.core import *
from parapy.geom import *

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
AIRFOIL_DIR = os.path.join(_module_dir, 'airfoils', '')


class Airfoil(GeomBase):  # note the use of FittedCurve as superclass
    chord = Input(1.)
    airfoil_name = Input("whitcomb")
    thickness_factor = Input(1.)
    mesh_deflection = Input(0.00001)
    tolerance = 0.00001

    @Attribute
    # Required input to the FittedCurve superclass
    def points(self):
        # Check whether the airfoil name string includes .dat already
        if self.airfoil_name.endswith(
                '.dat'):
            airfoil_file = self.airfoil_name
        else:
            airfoil_file = self.airfoil_name + '.dat'
        # Open the airfoil file and create a list of points
        file_path = os.path.join(AIRFOIL_DIR, airfoil_file)
        with open(file_path, 'r') as f:
            point_lst = []
            # The cartesian coordinates are directly interpreted as X and Z
            # coordinates
            for line in f:
                x, z = line.split(' ', 1)
                point_lst.append(self.position.translate(
                    # The x points are scaled according to the airfoil chord
                    # length
                    "x", float(x),
                    # The y points are scaled according to the thickness factor
                    "z", float(z)))
        return point_lst

    @Part(in_tree=False)
    def airfoil(self):
        return DynamicType(type=(Naca4AirfoilCurve if
                                 # NACA 4 if the airfoil input has 4 digits
                                 len(str(self.airfoil_name)) == 4
                                 # NACA 5 if the airfoil input has more digits
                                 else Naca5AirfoilCurve if
                                 len(str(self.airfoil_name)) == 5
                                 # Look for airfoil files if a longer name
                                 # is provided
                                 else FittedCurve),
                           # Provide required input for NACA airfoils
                           designation=str(self.airfoil_name),
                           # Provide points from .dat files as input for the
                           # fitted curve
                           points=self.points)

    @Part
    def curve(self):
        return ScaledCurve(curve_in=self.airfoil,
                           # Scale with the leading edge as reference point
                           reference_point=self.position.point,
                           # Scale times the chord in x direction, and also
                           # with the thickness factor in z direction to
                           # allow for multiple thickness airfoils
                           factor=(self.chord, 1,
                                   self.chord * self.thickness_factor))

    @Part
    def avl_section(self):
        return avl.SectionFromCurve(curve_in=self.curve)
