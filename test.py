import sys, os
# Add the 'classes' directory to the PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(__file__), './classes'))

from Robot import Robot

robot = Robot()