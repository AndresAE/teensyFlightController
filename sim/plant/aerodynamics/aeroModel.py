import numpy as np
from common.common import common


class aeroModel:
    def __init__(self, modelName="aero_model.npz"):
        self.data = np.load(modelName)

        # inputs
        self.cg_B_ft = np.array([np.nan, np.nan, np.nan])
        self.qBar_PSF = np.nan
        self.v_fps = np.nan
        self.alpha_d = np.nan
        self.beta_d = np.nan
        self.elevator_d = np.nan
        self.aileron_d = np.nan
        self.rudder_d = np.nan
        self.p_rps = np.nan
        self.q_rps = np.nan
        self.r_rps = np.nan

        # outputs
        self.coef = np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])
        self.f_B_lbf = np.array([np.nan, np.nan, np.nan])
        self.m_CG_B_ftlbf = np.array([np.nan, np.nan, np.nan])

        self.cmn = common()

        return

    def update(self):
        """Calculate mean aerodynamic chord for a trapezoidal wing."""

        # preproc
        self.pb2v = self.p_rps * self.data["bRef_ft"] / (2 * self.v_fps)
        self.qc2v = self.q_rps * self.data["cRef_ft"] / (2 * self.v_fps)
        self.rb2v = self.r_rps * self.data["bRef_ft"] / (2 * self.v_fps)

        for kk in range(0, 6):
            # baseline
            self.coef[kk] = self.interp2(self.data["alphaBP_d"], self.data["betaBP_d"],
                                         self.data["baseline"][:, :, kk], self.alpha_d, self.beta_d)

            # aileron
            self.coef[kk] += self.interp2(self.data["alphaBP_d"], self.data["aileronBP_d"],
                                          self.data["aileronInc"][:, :, kk], self.alpha_d, self.aileron_d)

            # elevator
            self.coef[kk] += self.interp2(self.data["alphaBP_d"], self.data["elevatorBP_d"],
                                          self.data["elevatorInc"][:, :, kk], self.alpha_d, self.elevator_d)

            # rudder
            self.coef[kk] += self.interp2(self.data["alphaBP_d"], self.data["rudderBP_d"],
                                          self.data["rudderInc"][:, :, kk], self.alpha_d, self.rudder_d)

            # roll rate
            self.coef[kk] += self.interp1(self.data["alphaBP_d"],
                                          self.data["rollRateDeriv"][:, kk], self.alpha_d) * self.pb2v

            # pitch rate
            self.coef[kk] += self.interp1(self.data["alphaBP_d"],
                                          self.data["pitchRateDeriv"][:, kk], self.alpha_d) * self.qc2v

            # yaw rate
            self.coef[kk] += self.interp1(self.data["alphaBP_d"],
                                          self.data["yawRateDeriv"][:, kk], self.alpha_d) * self.rb2v

        # calculate force/moment
        _f_S_lbf = self.qBar_PSF * \
            self.data["sRef_ft2"] * \
            np.array([-self.coef[0], self.coef[1], -self.coef[2]])
        _m_MRC_S_ftlbf = self.qBar_PSF * self.data["sRef_ft2"] * np.array(
            [self.coef[3] * self.data["bRef_ft"], self.coef[4] * self.data["cRef_ft"], self.coef[5] * self.data["bRef_ft"]])

        # transform to body
        self.f_B_lbf = np.transpose(self.cmn.stab2bodyTransformation(
            self.alpha_d)@_f_S_lbf.transpose())
        _m_MRC_B_ftlbf = np.transpose(self.cmn.stab2bodyTransformation(
            self.alpha_d)@_m_MRC_S_ftlbf.transpose())

        # shift to CG
        self.m_CG_B_ftlbf = _m_MRC_B_ftlbf - \
            np.cross(self.cg_B_ft-self.data["MRC_B_ft"], self.f_B_lbf)

        return

    def interp1(self, xv, y, xq):
        return np.interp(xq, xv, y)

    def interp2(self, xv, yv, Z, xq, yq):
        # xv: (Nx,) grid
        # yv: (Ny,) grid
        # Z:  (Nx, Ny) table

        # find indices
        i = np.searchsorted(xv, xq) - 1
        j = np.searchsorted(yv, yq) - 1

        # clamp
        i = np.clip(i, 0, len(xv)-2)
        j = np.clip(j, 0, len(yv)-2)

        # corners
        x1, x2 = xv[i], xv[i+1]
        y1, y2 = yv[j], yv[j+1]

        Q11 = Z[i,   j]
        Q21 = Z[i+1, j]
        Q12 = Z[i,   j+1]
        Q22 = Z[i+1, j+1]

        # weights
        tx = (xq - x1) / (x2 - x1)
        ty = (yq - y1) / (y2 - y1)

        # bilinear interpolation
        return (
            Q11*(1-tx)*(1-ty) +
            Q21*(tx)*(1-ty) +
            Q12*(1-tx)*(ty) +
            Q22*(tx)*(ty)
        )
