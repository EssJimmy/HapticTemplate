#include "Controllers.h"
#include "HelperFunctions.h"
#include <mmsystem.h>

const int NO_JOINTS = 3;

std::vector<double> Controllers::PIDController(const double PI, const double SAMPLE_TIME, 
				CHapticTemplateDlg* pMainWnd, bool iCSmc, double* qm) {
				std::vector<double> taum(NO_JOINTS);
				CString time;
				
				static double ti = 0;
				static double qm_1[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				const double alpha = 9.0 / 11.0;

				const double qmdf[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				const double qmd[NO_JOINTS] = { 0, 0, 0 };

				double t;
				double qd1 = 0;
				double qd2 = 0;
				double qd3 = 0;
				double dqd1 = 0;
				double dqd2 = 0;
				double dqd3 = 0;

				const double kp[NO_JOINTS] = { 1.5,3.5,2.8 };
				const double ki[NO_JOINTS] = { 0.2,0.25,0.25 };
				const double kd[NO_JOINTS] = { 0.1,0.2,0.2 };

				double dotePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double sq[NO_JOINTS] = { 0.0,0.0,0.0 };
				double qd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dqd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePosi[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double vel[NO_JOINTS] = { 0.0, 0.0, 0.0 };

				const double K1[NO_JOINTS] = { 11, 16, 12 };
				const double K2[NO_JOINTS] = { 0.05, 0.08, 0.06 };
				const double K3[NO_JOINTS] = { 0.05, 0.05, 0.05 };
				const double K4[NO_JOINTS] = { 0.16, 0.28, 0.26 };
				const double KV[NO_JOINTS] = { 0.08, 0.08, 0.08 };

				if (iCSmc) {
								ti = timeGetTime();
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));

								std::copy(qm, qm+NO_JOINTS, std::begin(qm_1));
				}

				t = (timeGetTime() - ti) / 1000.0;
				time.Format(_T("%f"), t);
				pMainWnd->m_time.SetWindowTextW(time);

				vel[0] = HelperFunctions::RED1(qm[0], SAMPLE_TIME);
				vel[1] = HelperFunctions::RED2(qm[1], SAMPLE_TIME);
				vel[2] = HelperFunctions::RED3(qm[2], SAMPLE_TIME);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < NO_JOINTS; i++) {
								ePos[i] = qm[i] - qd[i];
								dotePos[i] = vel[i] - dqd[i];
								sq[i] = dotePos[i] + K1[i] * HelperFunctions::Sign(ePos[i]) * (pow((abs(ePos[i])), alpha));
								ePosi[i] += ePos[i] * SAMPLE_TIME;

								taum[i] = -kp[i] * ePos[i] - ki[i] * ePosi[i] - kd[i] * dotePos[i];
				}

				std::free(dotePos); std::free(ePos); std::free(sq);
				std::free(qd); std::free(dqd); std::free(ePosi); std::free(vel);

				return taum;
}

std::vector<double> Controllers::ParraVegaController(const double PI, const double SAMPLE_TIME, 
				CHapticTemplateDlg* pMainWnd, bool iCSmc) {
				std::vector<double> taum(NO_JOINTS);
				CString time;

				static double ti = 0.0;
				double t = 0.0;

				const double gammaPV[NO_JOINTS] = { 0.4, 0.4, 0.4 };
				const double alphaPV[NO_JOINTS] = { 14.0, 14.0, 14.0 };
				const double kdPV[NO_JOINTS] = { 0.35, 0.35, 0.35 };

				static double sigmaPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				
				double dotePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dqd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dotqrPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double dotsigmaPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sdPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double srPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sqPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };

				if (iCSmc) {
								ti = timeGetTime();
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));
				}

				t = (timeGetTime() - ti) / 1000.0;
				time.Format(_T("%f"), t);
				pMainWnd->m_time.SetWindowTextW(time);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < NO_JOINTS; i++) {
								sigmaPV[i] += dotsigmaPV[i] * SAMPLE_TIME;
								dotqrPV[i] = dqd[i] - alphaPV[i] * ePos[i] + sdPV[i] - gammaPV[i] * sigmaPV[i];
								dotsigmaPV[i] = HelperFunctions::Sign(sqPV[i]);
								sPV[i] = dotePos[i] + alphaPV[i] * ePos[i];
								sqPV[i] = sPV[i];
								srPV[i] = sqPV[i] + gammaPV[i] * sigmaPV[i];

								taum[i] = -kdPV[i] * srPV[i];
				}

				std::free(dotePos); std::free(ePos); std::free(dqd);
				std::free(dotqrPV); std::free(dotsigmaPV); std::free(sPV); 
				std::free(sdPV); std::free(srPV); std::free(sqPV);

				return taum;
}

std::vector<double> Controllers::NLController(const double PI, const double SAMPLE_TIME,
				CHapticTemplateDlg* pMainWnd, bool iCSmc, double *qm) {
				std::vector<double> taum(NO_JOINTS);
				CString time;

				static double ti = 0;
				static double qm_1[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				static double sigma[NO_JOINTS] = { 0.0,0.0,0.0 };
				const double alpha = 9.0 / 11.0;

				const double qmdf[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				const double qmd[NO_JOINTS] = { 0, 0, 0 };

				double t;
				double qd1 = 0;
				double qd2 = 0;
				double qd3 = 0;
				double dqd1 = 0;
				double dqd2 = 0;
				double dqd3 = 0;

				double dotePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double sq[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dotqr[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dqm[NO_JOINTS] = { 0, 0, 0 };
				double s[NO_JOINTS] = { 0.0,0.0,0.0 };
				double qd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dqd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dotsigma[NO_JOINTS] = { 0.0,0.0,0.0 };
				double vel[NO_JOINTS] = { 0.0, 0.0, 0.0 };

				const double K1[NO_JOINTS] = { 11, 16, 12 };
				const double K2[NO_JOINTS] = { 0.05, 0.08, 0.06 };
				const double K3[NO_JOINTS] = { 0.05, 0.05, 0.05 };
				const double K4[NO_JOINTS] = { 0.16, 0.28, 0.26 };
				const double KV[NO_JOINTS] = { 0.08, 0.08, 0.08 };

				if (iCSmc) {
								ti = timeGetTime();
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));
				}

				t = (timeGetTime() - ti) / 1000.0;
				time.Format(_T("%f"), t);
				pMainWnd->m_time.SetWindowTextW(time);

				vel[0] = HelperFunctions::RED1(qm[0], SAMPLE_TIME);
				vel[1] = HelperFunctions::RED2(qm[1], SAMPLE_TIME);
				vel[2] = HelperFunctions::RED3(qm[2], SAMPLE_TIME);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < NO_JOINTS; i++) {
								dqm[i] = (qm[i] - qm_1[i]) * SAMPLE_TIME;
								ePos[i] = qm[i] - qd[i];
								dotePos[i] = vel[i] - dqd[i];
								sq[i] = dotePos[i] + K1[i] * HelperFunctions::Sign(ePos[i]) * (pow((abs(ePos[i])), alpha));
								dotqr[i] = dqd[i] - K1[i] * HelperFunctions::Sign(ePos[i]) * pow((abs(ePos[i])), alpha) - K2[i] * sigma[i];
								s[i] = dqm[i] - dotqr[i];
								dotsigma[i] = K3[i] * sq[i] + HelperFunctions::Sign(sq[i]);
								sigma[i] += dotsigma[i] * SAMPLE_TIME;

								taum[i] = K4[i] * tanh(s[i]);
				}

				std::free(dotePos); std::free(ePos); std::free(sq); std::free(dotqr);  std::free(dqm);
				std::free(s); std::free(qd); std::free(dqd); std::free(dotsigma); std::free(vel);

				return taum;
}