import numpy as np


class atmosphere:
    def __init__(self):
        # inputs
        self.altitudeMSL_ft = 0.0

        # atmosphere data
        self.gas_constant = 1716.5  # ft*lb/(slug*R)
        self.gamma = 1.4  # []

    def air_density(self):
        """return air density wrt altitude."""
        p_psf = self.pressure()  # psf
        t_R = self.temperature()  # R
        rho_slugft3 = p_psf / (self.gas_constant * t_R)  # lb/ft3
        return rho_slugft3

    def pressure(self):
        """return ambient pressure wrt altitude."""
        t_R = self.temperature()  # R
        if self.altitudeMSL_ft <= 36152:
            p_psf = 2116 * (t_R / 518.6) ** 5.256  # psf
        else:
            p_psf = 473.1 * np.exp(1.73 - 0.000048 *
                                   self.altitudeMSL_ft)  # psf
        return p_psf

    def speed_of_sound(self):
        """return speed of sound wrt altitude."""
        t_R = self.temperature()  # R
        a_fps = np.sqrt(self.gamma * self.gas_constant * t_R)  # [ft/s]
        return a_fps

    def viscosity(self):
        """return dynamic viscosity of air wrt altitude."""
        tRef_R = self.temperature()  # R
        t_R = self.temperature()  # R
        s_R = 198.72  # R
        mu_ref = 3.737 * 10 ** (-7)  # [slug/(ft*s)]
        mu = mu_ref * ((t_R / tRef_R) ** (3 / 2)) * \
            (tRef_R + s_R) / (t_R + s_R)  # [slug/(ft*s)]
        return mu

    def temperature(self):
        """return ambient temperature wrt altitude."""
        if self.altitudeMSL_ft <= 36152:
            t_R = 59 - 0.00356 * self.altitudeMSL_ft  # deg F
        else:
            t_R = -70  # deg F
        t_R = t_R + 459.7  # R
        return t_R
