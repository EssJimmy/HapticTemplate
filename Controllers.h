#pragma once
#include "stdafx.h"
#include "pch.h"
#include <vector>

class controllers {
public:
    static std::vector<std::vector<double>> pid_controller(const double pi, const double sample_time, bool i_c_smc,
        double* qm, static double ti);

    static std::vector<double> parra_vega_controller(const double pi, const double sample_time, bool i_c_smc,
        static double ti);

    static std::vector<std::vector<double>> nl_controller(const double pi, const double sample_time, bool i_c_smc,
        double *qm, static double ti);

    static std::vector<double> graph_trajectory(const double t, const double pi, const double* qm, const double* dote_pos, 
        const double* e_pos, const double* qd, const double* dqd, const double* vel, const double* dot_qr);
};