#include <iostream>
#include <cmath>

#define acceleration 10

float math()
{
    // 입력 P_1, P_0, T_1, T_0
    float P_1, P_0, T_1, T_0;

    std::cout << "Enter P_1: ";
    std::cin >> P_1;
    std::cout << "Enter P_0: ";
    std::cin >> P_0;
    std::cout << "Enter T_1: ";
    std::cin >> T_1;
    std::cout << "Enter T_0: ";
    std::cin >> T_0;

    float Vmax;

    float S = P_1 - P_0;
    int sign;
    if (S < 0)
    {
        S = -1 * S;
        sign = -1;
    }
    else
    {
        sign = 1;
    }

    float totalTime = T_1 - T_0;
    
    // 2차 방정식의 계수들
    float a = 1.0 / acceleration;
    float b = -T_1;
    float c = S;
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0)
    {
        std::cout << "No real solution for Vmax." << std::endl;
        return -1; // 실수 해가 없을 경우 -1 반환
    }
    else
    {
        // 2차 방정식의 해 구하기
        float Vmax1 = (-b + std::sqrt(discriminant)) / (2 * a);
        float Vmax2 = (-b - std::sqrt(discriminant)) / (2 * a);

        // 두 해 중 양수인 해 선택
        Vmax = (Vmax1 > 0) ? Vmax1 : Vmax2;

        std::cout << "Calculated Vmax: " << Vmax << std::endl;
    }

    if (S == 0)
    {
        // 정지
        return P_0;
    }
    else if (Vmax * Vmax / acceleration < S)
    {
        // 가속
        if (T_0 < Vmax / acceleration)
        {
            return P_0 + sign * 0.5 * acceleration * T_0 * T_0;
        }
        // 등속
        if (T_0 < S / Vmax)
        {
            return P_0 + sign * 0.5 * Vmax * Vmax / acceleration + Vmax * (T_0 - Vmax / acceleration); 
        }
        // 감속
        if (T_0 < Vmax / acceleration + S / Vmax)
        {
            return P_1 - sign * 0.5 * acceleration * (S / Vmax + Vmax / acceleration - T_0) * (S / Vmax + Vmax / acceleration - T_0);
        }
        else 
        {
            return P_1;
        }
    }

    return 0;
}

int main()
{
    float result = math();
    if (result != -1) {
        std::cout << "Result: " << result << std::endl;
    }
    return 0;
}
