#include "Status.h"

char buf[32];

void printStatusTM(Stream &serialPort, int now_ms, int endTime_ms, TST &tst, MSN &msn, IMU &imu, GPS &gps, NAV &nav, GDC &gdc, APT &apt)
{
    // print status to serial port (grouped to reduce calls)

    // time block
    {
        String s;
        s.reserve(50);
        s += "t: met_s=";
        s += String(now_ms / 1000.0);
        s += ", stp_ms=";
        s += String(endTime_ms - now_ms);
        serialPort.println(s);
    }

    // msn block
    serialPort.print("msn: phs=");
    serialPort.println(msn.missionPhase_e);

    // imu block
    {
        String s;
        s.reserve(140);
        s += "imu: ax_fps2=";
        s += String(imu.m_IMU.states.specificForceX_fps2);
        s += ", ay_fps2=";
        s += String(imu.m_IMU.states.specificForceY_fps2);
        s += ", az_fps2=";
        s += String(imu.m_IMU.states.specificForceZ_fps2);
        s += ", gx_dps=";
        s += String(imu.m_IMU.states.angularRateX_dps);
        s += ", gy_dps=";
        s += String(imu.m_IMU.states.angularRateY_dps);
        s += ", gz_dps=";
        s += String(imu.m_IMU.states.angularRateZ_dps);
        s += ", roll_d=";
        s += String(imu.m_IMU.euler.roll_d);
        s += ", pitch_d=";
        s += String(imu.m_IMU.euler.pitch_d);
        s += ", yaw_d=";
        s += String(imu.m_IMU.euler.yaw_d);
        serialPort.println(s);
    }

    // gps block
    {
        String s;
        s.reserve(160);
        s += "gps: timeStamp_s=";
        s += String(gps.m_GPS.timeStamp_s);
        s += ", latG_d=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.latG_d);
        s += buf;
        s += ", lon_d=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.lon_d);
        s += buf;
        s += ", HAE_ft=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.hae_ft);
        s += buf;
        s += ", vE_X_fps=";
        s += String(gps.m_vECEF.x_fps);
        s += ", vE_Y_fps=";
        s += String(gps.m_vECEF.y_fps);
        s += ", vE_Z_fps=";
        s += String(gps.m_vECEF.z_fps);
        s += ", fixQual=";
        s += String(gps.m_GPS.fixQuality);
        s += ", nSat=";
        s += String(gps.m_GPS.nSatellites);
        serialPort.println(s);
    }

    // gdc block
    {
        String s;
        s.reserve(90);
        s += "gdc: pbC_dps=";
        s += String(gdc.m_GDC.rollRateCmd_dps);
        s += ", nyC_B_fps2=";
        s += String(gdc.m_GDC.nyCmd_B_fps2);
        s += ", nzC_B_fps2=";
        s += String(gdc.m_GDC.nzCmd_B_fps2);
        s += ", pwrCmd_pct=";
        s += String(gdc.m_GDC.throttleCmd_pct);
        serialPort.println(s);
    }

    // apt block
    {
        String s;
        s.reserve(120);
        s += "apt: ";
        const int nEff = static_cast<int>(sizeof(apt.m_APT.effectorCmd) / sizeof(apt.m_APT.effectorCmd[0]));
        for (int ii = 0; ii < nEff; ii++)
        {
            s += "effC_";
            s += String(ii);
            s += ":";
            s += String(apt.m_APT.effectorCmd[ii]);
            if (ii < nEff - 1)
                s += ", ";
        }
        serialPort.println(s);
    }
}

void printStatus(Stream &serialPort, int now_ms, int endTime_ms, TST &tst, MSN &msn, IMU &imu, GPS &gps, NAV &nav, GDC &gdc, APT &apt)
{
    // print status to serial port (grouped to reduce calls)

    // time block
    {
        String s;
        s.reserve(160);
        s += "time: met_s=";
        s += String(now_ms / 1000.0);
        s += ", steptime_ms=";
        s += String(endTime_ms - now_ms);
        serialPort.println(s);
    }

    // msn block
    serialPort.print("msn: phase=");
    serialPort.println(msn.missionPhase_e);

    // tst block
    {
        String s;
        s.reserve(160);
        s += "tst: control_1_In=";
        s += String(tst.m_TST.controlIn[0]);
        s += ", control_2_In=";
        s += String(tst.m_TST.controlIn[1]);
        s += ", control_3_In=";
        s += String(tst.m_TST.controlIn[2]);
        s += ", control_4_In=";
        s += String(tst.m_TST.controlIn[3]);
        s += ", control_5_In=";
        s += String(tst.m_TST.controlIn[4]);
        serialPort.println(s);
    }

    // imu block
    {
        String s;
        s.reserve(160);
        s += "imu: ax_fps2=";
        s += String(imu.m_IMU.states.specificForceX_fps2);
        s += ", ay_fps2=";
        s += String(imu.m_IMU.states.specificForceY_fps2);
        s += ", az_fps2=";
        s += String(imu.m_IMU.states.specificForceZ_fps2);
        s += ", gx_dps=";
        s += String(imu.m_IMU.states.angularRateX_dps);
        s += ", gy_dps=";
        s += String(imu.m_IMU.states.angularRateY_dps);
        s += ", gz_dps=";
        s += String(imu.m_IMU.states.angularRateZ_dps);
        s += ", roll_d=";
        s += String(imu.m_IMU.euler.roll_d);
        s += ", pitch_d=";
        s += String(imu.m_IMU.euler.pitch_d);
        s += ", yaw_d=";
        s += String(imu.m_IMU.euler.yaw_d);
        serialPort.println(s);
    }

    // gps block
    {
        String s;
        s.reserve(240);
        s += "gps: timeStamp_s=";
        s += String(gps.m_GPS.timeStamp_s);
        s += ", latG_d=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.latG_d);
        s += buf;
        s += ", lon_d=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.lon_d);
        s += buf;
        s += ", HAE_ft=";
        snprintf(buf, sizeof(buf), "%.10g", gps.m_GPS.lla_ddft.hae_ft);
        s += buf;
        s += ", ECEF_X=";
        s += String(gps.m_ECEF.x_ft);
        s += ", ECEF_Y=";
        s += String(gps.m_ECEF.y_ft);
        s += ", ECEF_Z=";
        s += String(gps.m_ECEF.z_ft);
        s += ", vECEF_X_fps=";
        s += String(gps.m_vECEF.x_fps);
        s += ", vECEF_Y_fps=";
        s += String(gps.m_vECEF.y_fps);
        s += ", vECEF_Z_fps=";
        s += String(gps.m_vECEF.z_fps);
        s += ", fixQual=";
        s += String(gps.m_GPS.fixQuality);
        s += ", nSat=";
        s += String(gps.m_GPS.nSatellites);
        s += ", HDOP=";
        s += String(gps.m_GPS.hdop);
        serialPort.println(s);
    }

    // nav block
    {
        String s;
        s.reserve(160);
        s += "nav: uB_fps=";
        s += String(nav.m_NAV.u_fps2);
        s += ", vB_fps=";
        s += String(nav.m_NAV.v_fps2);
        s += ", wB_fps=";
        s += String(nav.m_NAV.w_fps2);
        serialPort.println(s);
    }

    // gdc block
    {
        String s;
        s.reserve(80);
        s += "gdc: pbCmd_dps=";
        s += String(gdc.m_GDC.rollRateCmd_dps);
        s += ", nyCmd_B_fps2=";
        s += String(gdc.m_GDC.nyCmd_B_fps2);
        s += ", nzCmd_B_fps2=";
        s += String(gdc.m_GDC.nzCmd_B_fps2);
        s += ", throttleCmd_pct=";
        s += String(gdc.m_GDC.throttleCmd_pct);
        serialPort.println(s);
    }

    // apt block
    {
        String s;
        s.reserve(160);
        s += "apt: ";
        const int nEff = static_cast<int>(sizeof(apt.m_APT.effectorCmd) / sizeof(apt.m_APT.effectorCmd[0]));
        for (int ii = 0; ii < nEff; ii++)
        {
            s += "effector";
            s += String(ii);
            s += "_cmd:";
            s += String(apt.m_APT.effectorCmd[ii]);
            if (ii < nEff - 1)
                s += ", ";
        }
        serialPort.println(s);
    }
}