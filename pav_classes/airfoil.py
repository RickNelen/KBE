import os.path
from abc import ABC

from parapy.geom import *
from parapy.core import *
from kbeutils.geom import *

_module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                           os.pardir))
AIRFOIL_DIR = os.path.join(_module_dir, 'airfoils', '')


class Airfoil(GeomBase):  # note the use of FittedCurve as superclass
    chord = Input(1.)
    airfoil_name = Input("whitcomb")
    thickness_factor = Input(1.)
    mesh_deflection = Input(0.0001)
    tolerance = 0.0001

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
                                 else Naca5AirfoilCurve),
                           designation=str(self.airfoil_name))

    @Part
    def curve(self):
        return ScaledCurve(curve_in=self.airfoil,
                           reference_point=self.position.point,
                           factor=(self.chord, 1,
                                   self.chord * self.thickness_factor))
