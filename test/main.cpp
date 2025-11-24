#include <iostream>
#include <cmath>

double a_ft = 20925646.33;
double f = 0.00335281066;
double e = 0.08181919078;
double deg2rad = 0.01745329251994;
double n_ft;
double ft2m = 0.3048;

struct busGPS{
    double latG_d;
    double lon_d;
    double hae_m;
    int fixQuality;
    int nSatellites;
    double hdop;
};

struct busLLA{
    double latG_d;
    double lon_d;
    double hae_ft;
};

struct busEcef{
    double x_ft;
    double y_ft;
    double z_ft;
};

busEcef ecef_ft;

double primeVerticalRadiusOfCurvature(double latG_d) {
    return a_ft / (std::pow(1 - std::pow(e, 2) * (std::pow(std::sin(deg2rad * latG_d), 2)), 0.5));
}

struct::busEcef lla2ecef(struct::busLLA lla_ddft) {
    n_ft = primeVerticalRadiusOfCurvature(lla_ddft.latG_d);
    ecef_ft.x_ft = (n_ft + lla_ddft.hae_ft) * std::cos(deg2rad * lla_ddft.latG_d) * std::cos(deg2rad * lla_ddft.lon_d);
    ecef_ft.y_ft = (n_ft + lla_ddft.hae_ft) * std::cos(deg2rad * lla_ddft.latG_d) * std::sin(deg2rad * lla_ddft.lon_d);
    ecef_ft.z_ft = ((1 - std::pow(e, 2))*n_ft + lla_ddft.hae_ft) * std::sin(deg2rad * lla_ddft.latG_d);
    return ecef_ft;
}

int main() {
    std::cout << "Hello, World!"<< std::endl;
    std::cout << "Hello Again, World!"<< std::endl;
    
    busGPS databus = {33.1, 106.1, 4000, 5, 4, 0.5};

    std::cout<<databus.hae_m+databus.latG_d<<std::endl;

    for (int ii = 0; ii <= 10; ii++) {
        std::cout<<ii<<std::endl;
    }

    std::cout<<primeVerticalRadiusOfCurvature(30)<<std::endl;
    busLLA lla_ddff = {33, -106, 4000};
    ecef_ft = lla2ecef(lla_ddff);
    std::cout<<ecef_ft.x_ft * ft2m<<','<<ecef_ft.y_ft * ft2m<<','<<ecef_ft.z_ft * ft2m<<std::endl;
    return 0;
}
