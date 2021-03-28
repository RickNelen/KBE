import os.path

_module_dir = os.path.dirname(__file__)
AIRFOIL_DIR = os.path.join(_module_dir, 'airfoils', '')

from .fuselage import Fuselage
from .airfoil import Airfoil
