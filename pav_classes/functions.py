# -----------------------------------------------------------------------------
# IMPORTS
# -----------------------------------------------------------------------------

from math import *

# -----------------------------------------------------------------------------
# FUNCTIONS
# -----------------------------------------------------------------------------

# These functions are used in multiple classes and files; thus, they are
# defined here for easy imports


def generate_warning(warning_header, message):
    # This function creates a messagebox with a warning header and a message
    from tkinter import Tk, messagebox
    window = Tk()
    window.withdraw()
    messagebox.showwarning(warning_header, message)


def chord_length(root_chord, tip_chord, span_position):
    # Determine the chord length based on the span-wise location of this
    # profile, the root chord of the wing and the tip chord of the wing
    return root_chord - (root_chord - tip_chord) * span_position


def sweep_to_sweep(x_over_c_start, sweep_start, x_over_c_end, aspect_ratio,
                   taper_ratio):
    # Determine the sweep at a certain chord-wise position based on a given
    # sweep at another chord-wise position
    tan_sweep_end = (tan(sweep_start) - 4 / aspect_ratio
                     * (x_over_c_end - x_over_c_start)
                     * (1 - taper_ratio) / (1 + taper_ratio))
    return atan(tan_sweep_end)