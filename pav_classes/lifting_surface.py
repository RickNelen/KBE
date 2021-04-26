# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from math import radians, tan
from parapy.geom import *
from parapy.core import *
from .functions import *
import kbeutils.avl as avl
from .airfoil import Airfoil


# -----------------------------------------------------------------------------
# LIFTING SURFACE CLASS
# -----------------------------------------------------------------------------


class LiftingSurface(GeomBase):
    # -------------------------------------------------------------------------
    # INPUTS
    # -------------------------------------------------------------------------

    # Name the instance of the lifting surface
    name = Input()

    # Number of sections along the span for which the profiles are defined
    number_of_profiles = Input(4)
    airfoils = Input(['4415', '4415', '4405', 'whitcomb'])

    # Prevent warnings from popping up until the design is converged
    hide_warnings = Input(False)

    # Define if the wing shall be mirrored
    is_mirrored = Input(True)

    # Geometric parameters of the wing
    span = Input(10.)
    aspect_ratio = Input(6.)
    taper_ratio = Input(0.3)

    # Thickness factors to adjust the thickness
    thickness_factor_root = Input(1.)
    thickness_factor_tip = Input(1.)

    # Geometric angles
    # The sweep is defined at the quarter chord point
    sweep = Input(20)
    # The incidence angle is the angle between the fuselage longitudinal
    # axis and the root chord line
    incidence_angle = Input(2)
    # The twist shows the twist angle of the tip profile around the quarter
    # chord point with respect to the root profile
    twist = Input(-5)
    # The dihedral angle shows the angle of the quarter chord line with
    # respect to the horizontal plane
    dihedral = Input(0)

    # Catch whether the inputted span is actually the semi-span or the
    # complete span
    @Input
    def semi_span(self):
        return (self.span / 2. if self.is_mirrored is True
                else self.span)

    # Adjust the span fractions of the profiles if preferred
    @Input
    def span_fraction_of_profiles(self):
        return [1. / (self.number_of_profiles - 1) * index for index in
                range(self.number_of_profiles)]

    # Adjust the chord length to generate non-trapezoidal wings
    @Input
    def chord_factor(self):
        return [1] * self.number_of_profiles

    # -------------------------------------------------------------------------
    # ATTRIBUTES
    # -------------------------------------------------------------------------

    @Attribute
    def profile_shape(self):
        # Although the program can deal with the number of defined airfoils
        # not being equal to the number of profiles, the user should be made
        # aware of this incompatibility through a warning
        if len(self.airfoils) == int(self.number_of_profiles):
            return self.airfoils
        elif len(self.airfoils) < self.number_of_profiles:
            message = 'The number of airfoils that is provided is lower than' \
                      ' the number of sections at which these should ' \
                      'be defined. The latest defined airfoil is used for ' \
                      'all sections further outboard. Please adjust this in ' \
                      'the part {}.'.format(self.name)
            generate_warning('Warning: shape adjusted', message)
            return self.airfoils
        else:
            message = 'The number of airfoils that is provided is larger ' \
                      'than the number of sections at which these should ' \
                      'be defined. Only those airfoils that are assigned to ' \
                      'a section are used, the remainder is ignored. Please ' \
                      'adjust this in the part {}.'.format(self.name)
            generate_warning('Warning: shape adjusted', message)
            return self.airfoils

    @Attribute
    def mean_aerodynamic_chord(self):
        # Compute the mean aerodynamic chord
        return (2. / 3 * self.root_chord
                * (1 + self.taper_ratio + self.taper_ratio ** 2)
                / (1 + self.taper_ratio))

    @Attribute
    # Compute the y-coordinate of the mean aerodynamic chord
    def lateral_position_of_mean_aerodynamic_chord(self):
        return (self.span / 6
                * (self.root_chord + 2 * self.tip_chord)
                / (self.root_chord + self.tip_chord))

    @Attribute
    def surface_area(self):
        # Compute surface area from span and aspect ratio
        return self.span ** 2 / self.aspect_ratio

    @Attribute
    def root_chord(self):
        # Compute the root chord of a trapezoidal wing
        return 2. * self.surface_area / (self.span * (1 + self.taper_ratio))

    @Attribute
    def tip_chord(self):
        # Compute the tip chord from the root chord and taper ratio
        return self.root_chord * self.taper_ratio

    @Attribute
    def profile_chords(self):
        # Compute the 'regular' chord lengths of a trapezoidal wing based on
        # span-wise location
        return [self.root_chord - (self.root_chord - self.tip_chord) * eta
                for eta in self.span_fraction_of_profiles]

    @Attribute
    def profile_thickness_factor(self):
        # Interpolate thickness factors at root and tip for all sections
        return [self.thickness_factor_root - eta
                * (self.thickness_factor_root - self.thickness_factor_tip)
                for eta in self.span_fraction_of_profiles]

    @Attribute
    def profile_locations(self):
        # Define the twist angle per section
        twist = [radians(self.incidence_angle +
                         self.span_fraction_of_profiles[index] * self.twist)
                 for index in range(len(self.span_fraction_of_profiles))]
        # Convert the sweep angle to radians
        sweep = radians(self.sweep)
        # Convert the dihedral angle to radians
        dihedral = radians(self.dihedral)
        # Determine the quarter chord length for each section
        quarter_chord = [self.profile_chords[index]
                         * self.chord_factor[index] / 4
                         for index in
                         range(len(self.span_fraction_of_profiles))]

        # Provide the locations of the quarter chord points for all sections
        ref_positions = [self.position.translate(
            # Translate in x direction
            self.position.Vx,
            self.span_fraction_of_profiles[index] * self.semi_span
            * tan(sweep),
            # Translate in y direction
            self.position.Vy,
            self.span_fraction_of_profiles[index] * self.semi_span,
            # Translate in z direction
            self.position.Vz,
            self.span_fraction_of_profiles[index] * self.semi_span
            * tan(dihedral))
            for index in range(len(self.span_fraction_of_profiles))]

        # Position each section relative to the quarter chord point at the root
        return [rotate(translate(ref_positions[index],
                                 # Move the start of the airfoil from the
                                 # quarter chord point to the LE
                                 self.position.Vx, - quarter_chord[index]),

                       # Rotate the section around the quarter chord point
                       # to apply twist
                       self.position.Vy, twist[index],
                       ref=ref_positions[index].point)
                for index in range(len(ref_positions))]

    @Attribute
    def profile_airfoils(self):
        # The total number of profiles that should be defined
        numbers = [number for number in range(self.number_of_profiles)]
        # Generate a list with the length of the provided number of airfoils
        name = len(self.profile_shape) - 1
        # If an insufficient number of profiles is defined by name, then just
        # use the latest profile for the remainder
        return [self.profile_shape[name] if number > name
                else self.profile_shape[number] for number in numbers]

    # -------------------------------------------------------------------------
    # PARTS
    # -------------------------------------------------------------------------

    # profiles returns the airfoils at the various sections along the
    # surface; surface itself returns the lofted shape of the lifting
    # surface; if a wing is considered and the shape should be mirrored,
    # mirrored provides the other half; avl_surface generates the surface
    # used in the AVL analysis

    @Part
    def profiles(self):
        return Airfoil(quantify=self.number_of_profiles,
                       airfoil_name=self.profile_airfoils[child.index],
                       # Provide chord with scaling factor if required
                       chord=(self.profile_chords[child.index]
                              * self.chord_factor[child.index]),
                       thickness_factor=self.profile_thickness_factor[
                           child.index],
                       position=self.profile_locations[child.index])

    @Part
    def surface(self):
        return LoftedSolid(profiles=[profile.curve for profile in
                                     self.profiles],
                           ruled=True)

    @Part
    def mirrored(self):
        return MirroredShape(quantify=self.number_of_profiles - 1,
                             shape_in=self.surface.faces[child.index],
                             reference_point=self.position.point,
                             vector1=self.position.Vx,
                             vector2=self.position.Vz,
                             suppress=not self.is_mirrored)

    @Part
    def avl_surface(self):
        return avl.Surface(name=self.name,
                           n_chordwise=12,
                           chord_spacing=avl.Spacing.cosine,
                           n_spanwise=20,
                           span_spacing=avl.Spacing.cosine,
                           y_duplicate=self.position.point[1]
                           if self.is_mirrored else None,
                           sections=[section.avl_section for section in
                                     self.profiles])
