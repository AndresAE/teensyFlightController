import numpy as np
from common.common import common


class fcs:
    def __init__(self, dt_s=0.01):
        # inputs
        self.wBI_B_dps = np.array([0.0, 0.0, 0.0])
        self.specificForce_fps2 = np.array([0.0, 0.0, 0.0])

        self.pC_B_dps = 0.0
        self.nzC_B_fps2 = 0.0
        self.nyC_B_fps2 = 0.0

        self.ki_p = 2.0
        self.kd_p = 1.0

        self.kp_nz = -2.0
        self.ki_q = 1.0
        self.kd_q = -3.0

        self.kp_ny = 3.0
        self.ki_r = 1.0
        self.kd_r = 3.0

        self.aileronLimit = np.array([-20, 20])
        self.elevatorLimit = np.array([-30, 30])
        self.rudderLimit = np.array([-30, 30])

        self.rollIntegral = 0.0
        self.pitchIntegral = 0.0
        self.yawIntegral = 0.0

        self.pErrPrev_dps = 0.0
        self.qErrPrev_dps = 0.0
        self.rErrPrev_dps = 0.0

        self.aileron_d = 0.0
        self.elevator_d = 0.0
        self.rudder_d = 0.0

        self.dt_s = dt_s

        self.cmn = common()
        return

    def step(self):
        # middle loops
        _pC_B_dps = self.pC_B_dps
        _qC_B_dps = self.kp_nz * (self.nzC_B_fps2 - self.specificForce_fps2[2])
        _rC_B_dps = self.kp_ny * (self.nyC_B_fps2 - self.specificForce_fps2[1])

        # inner loops
        _pErr_dps = _pC_B_dps - self.wBI_B_dps[0]
        _qErr_dps = _qC_B_dps - self.wBI_B_dps[1]
        _rErr_dps = _rC_B_dps - self.wBI_B_dps[2]

        # integral
        self.rollIntegral += self.dt_s * (_pErr_dps + self.pErrPrev_dps) / 2
        self.pitchIntegral += self.dt_s * (_qErr_dps + self.qErrPrev_dps) / 2
        self.yawIntegral += self.dt_s * (_rErr_dps + self.rErrPrev_dps) / 2

        self.pErrPrev_dps = _pErr_dps
        self.qErrPrev_dps = _qErr_dps
        self.rErrPrev_dps = _rErr_dps

        self.aileron_d = self.kd_p * \
            (self.rollIntegral*self.ki_p-self.wBI_B_dps[0])
        self.elevator_d = self.kd_q * \
            (self.pitchIntegral*self.ki_q-self.wBI_B_dps[1])
        self.rudder_d = self.kd_r * \
            (self.yawIntegral*self.ki_r-self.wBI_B_dps[2])

        # limiting
        if self.aileron_d < self.aileronLimit[0]:
            self.aileron_d = self.aileronLimit[0]
        elif self.aileron_d > self.aileronLimit[1]:
            self.aileron_d = self.aileronLimit[1]

        if self.elevator_d < self.elevatorLimit[0]:
            self.elevator_d = self.elevatorLimit[0]
        elif self.elevator_d > self.elevatorLimit[1]:
            self.elevator_d = self.elevatorLimit[1]

        if self.rudder_d < self.rudderLimit[0]:
            self.rudder_d = self.rudderLimit[0]
        elif self.rudder_d > self.rudderLimit[1]:
            self.rudder_d = self.rudderLimit[1]
        return
