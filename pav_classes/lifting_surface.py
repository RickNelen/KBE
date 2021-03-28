from math import radians, tan
from parapy.geom import *
from parapy.core import *
from kbeutils.geom import *
from .airfoil import Airfoil


class LiftingSurface(LoftedSolid):  # note use of loftedSolid as superclass
    airfoil_root = Input("whitcomb")
    airfoil_tip = Input("simm_airfoil")  #: :type: string

    w_c_root = Input(6.)
    w_c_tip = Input(2.3)
    t_factor_root = Input(1.)
    t_factor_tip = Input(1.)

    w_semi_span = Input(10.)
    sweep = Input(20)
    twist = Input(-5)

    mov_start = Input(
        0.7)  #: spanwise position of movable inboard section, as % of lifting surface span
    mov_end = Input(
        0.95)  #: spanwise position of movable outboard section, as % of lifting surface span
    h_c_fraction = Input(0.8)  # movable hinge position, as % of chord
    s_c_fraction1 = Input(0.85)  # movable front spar position, as % of chord
    s_c_fraction2 = Input(0.9)  # movable back spar position, as % of chord

    @Attribute  # required input for the superclass LoftedSolid
    def profiles(self):
        return [self.root_airfoil, self.tip_airfoil]

    @Part
    def root_airfoil(self):
        return Airfoil(airfoil_name=self.airfoil_root,
                       chord=5.)

    @Part
    def tip_airfoil(self):
        return Airfoil(airfoil_name=self.airfoil_tip,
                       chord=2.,
                       position=translate(self.position,
                                          x=self.w_semi_span *
                                            tan(radians(self.sweep)),
                                          y=self.w_semi_span))

    #@Part
    #def lofted_surf(self):
    #    return LoftedSurface(profiles=self.profiles)
