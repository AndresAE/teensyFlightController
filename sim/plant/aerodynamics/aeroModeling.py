import numpy as np


class aeroModeling:
    def __init__(self):
        return

    def meanAerodynamicChord(self, cRoot, taperRatio):
        """Calculate mean aerodynamic chord for a trapezoidal wing."""
        return (2/3) * cRoot * (1 + taperRatio + (taperRatio)**2) / (1 + taperRatio)

    def meanAerodynamicChordSpanwise(self, span, cRoot, taperRatio):
        """Calculate mean aerodynamic chord at a given spanwise location."""
        return (span / 6) * (1 + 2 * taperRatio) / (1 + taperRatio)

    def polhamus(self, aspectRatio, cla=2*np.pi):
        """Polhamus approximation for lift curve slope of a 3D wing."""
        k = 1+(8.2-aspectRatio*0.22)/100
        return cla * aspectRatio / (np.sqrt(4 + (aspectRatio**2)/k) + 2)

    def planformArea(self, span, cRoot, cTip):
        """Calculate wing area."""
        return 0.5*(cRoot+cTip)*span

    def aspectRatio(self, span, area):
        """Calculate aspect ratio."""
        return (span**2)/area

    def taperRatio(self, cRoot, cTip):
        """Calculate aspect ratio."""
        return cTip/cRoot
