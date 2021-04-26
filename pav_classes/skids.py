# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from parapy.geom import *
from parapy.core import *

# -----------------------------------------------------------------------------
# CLASS SKID
# -----------------------------------------------------------------------------


class Skid(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # Get the dimensions of the skids
    skid_length = Input(2.)
    skid_width = Input(0.3)
    skid_height = Input(0.2)

    @Part(in_tree=False)
    def skid_profile(self):
        # Create profiles at the front and rear end of the skid, based on
        # simple ellipses; the width is larger than the height
        return Ellipse(quantify=2,
                       major_radius=self.skid_width / 2,
                       minor_radius=self.skid_height / 2,
                       # The skids need to be rotated first around the
                       # z-axis and then around the y-axis
                       position=rotate90(rotate90(translate(self.position,
                                                            self.position.Vx,
                                                            self.skid_length
                                                            * child.index),
                                                  self.position.Vz),
                                         self.position.Vy))

    @Part
    def skid(self):
        # Create the shape of the skid
        return LoftedSolid(profiles=self.skid_profile,
                           color='silver')

