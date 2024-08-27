#pragma once
#include "pch.h"
#include "HapticTemplateDlg.h"
#include <vector>

class Controllers {
public:
    static std::vector<double> PIDController(const double PI, const double SAMPLE_TIME, 
        CHapticTemplateDlg *pMainWnd, bool iCSmc, double* qm);

    static std::vector<double> ParraVegaController(const double PI, const double SAMPLE_TIME,
        CHapticTemplateDlg* pMainWnd, bool iCSmc);

    static std::vector<double> NLController(const double PI, const double SAMPLE_TIME,
        CHapticTemplateDlg* pMainWnd, bool iCSmc, double *qm);
};