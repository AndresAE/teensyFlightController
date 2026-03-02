import numpy as np


class massModeling:
    def __init__(self):
        return

    def axialMassMomentInertiaCylinder(self, mass, radius):
        """Calculate axial mass moment of inertia for a cylinder."""
        return (1/2)*mass*radius**2

    def longitudinalMassMomentInertiaCylinder(self, mass, radius, length):
        """Calculate longitudinal mass moment of inertia for a cylinder."""
        return ((1/4)*mass*radius**2)+((1/12)*mass*length**2)
