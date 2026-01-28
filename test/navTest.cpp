#include "../src/common/Common.h"
#include "iostream"

Common C;

int main()
{

    Common::matrix mat_A = {3, 3, {{2.0f, 1.0f, 3.0f}, {0.0f, 2.0f, 4.0f}, {1.0f, 1.0f, 2.0f}}};
    Common::matrix mat_B = {3, 1, {{1.0f}, {2.0f}, {3.0f}}};
    Common::matrix mat_C = {3, 1, {{5.0f}, {6.0f}, {7.0f}}};
    Common::matrix mat_D = C.matrixMultiply(mat_A, mat_B);

    std::cout << "Result of matrix multiplication:\n";
    for (int i = 0; i < mat_D.rows; i++)
    {
        for (int j = 0; j < mat_D.cols; j++)
        {
            std::cout << mat_D.m[i][j] << " ";
        }
        std::cout << "\n";
    }

    Common::matrix mat_E = C.matrixAdd(mat_B, mat_C, true);

    std::cout << "Result of matrix addition:\n";
    for (int i = 0; i < mat_E.rows; i++)
    {
        for (int j = 0; j < mat_E.cols; j++)
        {
            std::cout << mat_E.m[i][j] << " ";
        }
        std::cout << "\n";
    }

    Common::matrix mat_F = C.matrixTranspose(mat_A);
    std::cout << "Result of matrix transpose:\n";
    for (int i = 0; i < mat_F.rows; i++)
    {
        for (int j = 0; j < mat_F.cols; j++)
        {
            std::cout << mat_F.m[i][j] << " ";
        }
        std::cout << "\n";
    }

    Common::matrix mat_G = C.matrixInverse(mat_A);
    std::cout << "Result of matrix inverse:\n";
    for (int i = 0; i < mat_G.rows; i++)
    {
        for (int j = 0; j < mat_G.cols; j++)
        {
            std::cout << mat_G.m[i][j] << " ";
        }
        std::cout << "\n";
    }

    Common::matrix mat_H = C.crossProductMatrix(mat_B, mat_C);
    std::cout << "Result of cross product:\n";
    for (int i = 0; i < mat_H.rows; i++)
    {
        for (int j = 0; j < mat_H.cols; j++)
        {
            std::cout << mat_H.m[i][j] << " ";
        }
        std::cout << "\n";
    }

    Common::busLLA lla = {37.7749, -122.4194, 100.0};
    Common::busEcef ecef = C.lla2ecef(lla);
    Common::busLLA lla2 = C.ecef2lla(ecef);
    std::cout << "Original LLA: " << lla.latG_d << ", " << lla.lon_d << ", " << lla.hae_ft << "\n";
    std::cout << "Converted ECEF: " << ecef.x_ft << ", " << ecef.y_ft << ", " << ecef.z_ft << "\n";
    std::cout << "Converted back to LLA: " << lla2.latG_d << ", " << lla2.lon_d << ", " << lla2.hae_ft << "\n";

    return 0;
}

// g++ -std=c++11 navTest.cpp src/Common/Common.cpp -I src/common -o test_app
// ./test_app
// Expected output:
// Result of matrix multiplication:
// 38
// 92
// 146