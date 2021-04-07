from math import radians, tan
from parapy.geom import *
from parapy.core import *
from kbeutils.geom import *
import kbeutils.avl as avl
from .airfoil import Airfoil


class LiftingSurface(GeomBase):
    # Name the instance of the lifting surface
    name = Input()

    # Number of sections along the span for which the profiles are defined
    number_of_profiles = Input(4)
    airfoils = Input(['4415', 'whitcomb', '4415', '4405'])

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
    sweep = Input(20)
    incidence_angle = Input(2)
    twist = Input(-5)
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

    # Compute surface area from span and aspect ratio
    @Attribute
    def surface_area(self):
        return self.span ** 2 / self.aspect_ratio

    @Attribute
    def root_chord(self):
        return 2. * self.surface_area / (self.span * (1 + self.taper_ratio))

    @Attribute
    def tip_chord(self):
        return self.root_chord * self.taper_ratio

    # Compute the 'regular' chord lengths of a trapezoidal wing based on
    # span-wise location
    @Attribute
    def profile_chords(self):
        return [self.root_chord - (self.root_chord - self.tip_chord) * eta for
                eta in self.span_fraction_of_profiles]

    # Interpolate thickness factors at root and tip for all sections
    @Attribute
    def profile_thickness_factor(self):
        return [self.thickness_factor_root - (
                self.thickness_factor_root - self.thickness_factor_tip) * eta
                for eta in self.span_fraction_of_profiles]

    @Attribute
    def profile_locations(self):
        # Define the twist angle per section
        twist = [radians(self.incidence_angle +
                         self.span_fraction_of_profiles[index] * self.twist)
                 for index in range(len(self.span_fraction_of_profiles))]
        sweep = radians(self.sweep)
        dihedral = radians(self.dihedral)
        # Position each section
        return [rotate(translate(self.position,
                                 # Translate in the longitudinal direction
                                 self.position.Vx,
                                 self.span_fraction_of_profiles[index]
                                 * self.semi_span * tan(sweep),
                                 # Translate in the span-wise direction
                                 self.position.Vy,
                                 self.span_fraction_of_profiles[index]
                                 * self.semi_span,
                                 # Translate in the vertical direction
                                 self.position.Vz,
                                 self.span_fraction_of_profiles[index]
                                 * tan(dihedral)),
                       # Then rotate the section
                       self.position.Vy, twist[index])
                for index in range(len(self.span_fraction_of_profiles))]

    @Attribute
    def profile_airfoils(self):
        # The total number of profiles that should be defined
        numbers = [number for number in range(self.number_of_profiles)]
        # Generate a list with the length of the provided number of airfoils
        name = len(self.airfoils) - 1
        # If an insufficient number of profiles is defined by name, then just
        # use the latest profile for the remainder
        return [self.airfoils[name] if number > name else self.airfoils[
            number] for number in numbers]

    # -------------------------
    # TO DO: ADD WARNING THAT THE USER SHOULD ADD MORE PROFILES?
    # -------------------

    @Part
    def profiles(self):
        return Airfoil(quantify=self.number_of_profiles,
                       airfoil_name=self.profile_airfoils[child.index],
                       # Provide chord with scaling factor if required
                       chord=self.profile_chords[child.index] *
                             self.chord_factor[child.index],
                       thickness_factor=self.profile_thickness_factor[
                           child.index],
                       position=self.profile_locations[child.index]
                       )

    @Part
    def surface(self):
        return LoftedSolid(profiles=[profile.curve for profile in
                                     self.profiles],
                           ruled=True)

    @Part
    def mirrored(self):
        return MirroredSurface(quantify=self.number_of_profiles - 1,
                               surface_in=
                               self.surface.faces[child.index],
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
