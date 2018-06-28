#include "PID.h"
#include <iostream>
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

}

PID::~PID() {

}

void PID::Init(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;

    p_error = 0.0;
    d_error = 0.0;
    i_error = 0.0;

    step = 0;
    min_step = 50;

    max_tolerance = 0.00005;
    min_tolerance = 0.0000005;

    dKp = p > min_tolerance ? p * 0.1 : 1.0;
    dKi = i > min_tolerance ? i * 0.1 : 1.0;
    dKd = d > min_tolerance ? d * 0.1 : 1.0;

    best_error = std::numeric_limits<double>::max();
    squared_error = 0.0;

    twiddle_state = 0;
    twiddle_param = 2;
}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;

    p_error = cte;

    i_error += cte;

    ++step;

    if(step > min_step) {
        squared_error = cte * cte;
    }
}

double PID::TotalError() {
    Twiddle();

    std::cout << "K: [" << Kp << "," << Ki << "," << Kd << "]" << std::endl;
    std::cout << "e: [" << p_error << "," << i_error << "," << d_error << "]" << endl;

    return -p_error * Kp - i_error * Ki - d_error * Kd;
}

void PID::Twiddle() {
    if (step > 2 * min_step) {
        double Ksum = Kp + Ki + Kd;
        if (Ksum > min_tolerance && Ksum < max_tolerance) {
            return;
        }

        double cur_error;
        if(twiddle_state == 0) {
            std::cout << "K: [" << Kp << "," << Ki << "," << Kd << "]" << std::endl;
            std::cout << "dK: [" << dKp << "," << dKi << "," << dKd << "]" << endl;

            twiddle_param = (twiddle_param + 1) % 3;
            if(twiddle_param == 0) {
                Kp += dKp;
            }
            else if(twiddle_param == 1) {
                Ki += dKi;
            }
            else if(twiddle_param == 2) {
                Kd += dKd;
            }

            squared_error = 0.0;
            step = 0;
            p_error = 0.0;
            i_error = 0.0;
            d_error = 0.0;

            twiddle_state = 1;
        }
        else if (twiddle_state == 1 ) {
            cur_error = squared_error / step;
            if(cur_error < best_error) {
                best_error = cur_error;
                if(twiddle_param == 0) {
                    dKp *= 1.1;
                }
                else if(twiddle_param == 1) {
                    dKi *= 1.1;
                }
                else if(twiddle_param == 2) {
                    dKd *= 1.1;
                }

                twiddle_state = 0;
            }
            else {
                if(twiddle_param == 0) {
                    Kp -= 2.0 * dKp;
                }
                else if(twiddle_param == 1) {
                    Ki -= 2.0 * dKi;
                }
                else if(twiddle_param == 2) {
                    Kd -= 2.0 * dKd;
                }

                squared_error = 0.0;
                step = 0;
                p_error = 0.0;
                i_error = 0.0;
                d_error = 0.0;

                twiddle_state = -1;
            }
        }
        else if (twiddle_state == -1)
        {
            cur_error = squared_error / step;
            if(cur_error < best_error) {
                best_error = cur_error;
                if(twiddle_param == 0) {
                    dKp *= 1.1;
                }
                else if(twiddle_param == 1) {
                    dKi *= 1.1;
                }
                else if(twiddle_param == 2) {
                    dKd *= 1.1;
                }
            }
            else {
                if(twiddle_param == 0) {
                    Kp += dKp;
                    dKp *= 1.1;
                }
                else if(twiddle_param == 1) {
                    Ki += dKi;
                    dKi *= 1.1;
                }
                else if(twiddle_param == 2) {
                    Kd += dKd;
                    dKd *= 1.1;
                }
            }

            twiddle_state = 0;
        }
    }
}