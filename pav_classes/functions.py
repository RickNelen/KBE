
from math import *


def generate_warning(warning_header, message):
    from tkinter import Tk, mainloop, X, messagebox

    window = Tk()
    window.withdraw()
    messagebox.showwarning(warning_header, message)


def chord_length(root_chord, tip_chord, span_position):
    return root_chord - (root_chord - tip_chord) * span_position


def sweep_to_sweep(x_over_c_start, sweep_start, x_over_c_end, aspect_ratio,
                   taper_ratio):
    tan_sweep_end = (tan(sweep_start) - 4 / aspect_ratio
                     * (x_over_c_end - x_over_c_start)
                     * (1 - taper_ratio) / (1 + taper_ratio))
    return atan(tan_sweep_end)