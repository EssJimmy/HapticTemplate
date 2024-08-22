#pragma once
#include "pch.h"
#include "HapticTemplateDlg.h"
#include <vector>

class Controllers {
public:
    static std::vector<double> Controllers::PIDController(const double PI, const double SAMPLE_TIME, double *qm, 
        CHapticTemplateDlg *pMainWnd, bool iCSmc);

    static std::vector<double> Controllers::ParraVegaController(const double PI, const double SAMPLE_TIME,
        CHapticTemplateDlg* pMainWnd, bool iCSmc);
};