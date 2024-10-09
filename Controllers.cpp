#include "pch.h"
#include "Controllers.h"
#include "HelperFunctions.h"
#include <mmsystem.h>

constexpr int no_joints = 3;
std::vector<double> tau(no_joints, 0.0);

std::vector<std::vector<double>> controllers::pid_controller(const double pi, const double sample_time, const bool i_c_smc, double* qm, 
				static double ti) {
				
				static double qm_1[no_joints] = { 0, 90.0 * pi / 180.0, -90.0 * pi / 180.0 };
				constexpr double dot_qr[no_joints] = { 0.0,0.0,0.0 };
				constexpr double alpha = 9.0 / 11.0;

				constexpr double kp[no_joints] = { 1.5,3.5,2.8 };
				constexpr double ki[no_joints] = { 0.2,0.25,0.25 };
				constexpr double kd[no_joints] = { 0.1,0.2,0.2 };

				double dote_pos[no_joints] = { 0.0,0.0,0.0 };
				double e_pos[no_joints] = { 0.0,0.0,0.0 };
				double sq[no_joints] = { 0.0,0.0,0.0 };
				double qd[no_joints] = { 0.0,0.0,0.0 };
				double dqd[no_joints] = { 0.0,0.0,0.0 };
				double e_pos_i[no_joints] = { 0.0, 0.0, 0.0 };
				double vel[no_joints] = { 0.0, 0.0, 0.0 };

				constexpr double k1[no_joints] = { 11, 16, 12 };

				if (i_c_smc) {
								ti = timeGetTime();
								std::copy_n(qm, no_joints, std::begin(qm_1));
				}

				const double t = (timeGetTime() - ti) / 1000.0;

				vel[0] = HelperFunctions::RED1(qm[0], sample_time);
				vel[1] = HelperFunctions::RED2(qm[1], sample_time);
				vel[2] = HelperFunctions::RED3(qm[2], sample_time);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
								e_pos[i] = qm[i] - qd[i];
								dote_pos[i] = vel[i] - dqd[i];
								sq[i] = dote_pos[i] + k1[i] * HelperFunctions::Sign(e_pos[i]) * (pow((abs(e_pos[i])), alpha));
								e_pos_i[i] += e_pos[i] * sample_time;

								tau[i] = -kp[i] * e_pos[i] - ki[i] * e_pos_i[i] - kd[i] * dote_pos[i];
				}

				std::vector<std::vector<double>> tau_graph_data = { tau,
				    graph_trajectory(t, pi, qm, dote_pos, e_pos, qd, dqd, vel, dot_qr) };

				return tau_graph_data;
}

std::vector<std::vector<double>> controllers::parra_vega_controller(const double pi, const double sample_time, const bool i_c_smc,
				static double ti) {
    constexpr double gamma_pv[no_joints] = { 0.4, 0.4, 0.4 };
    constexpr double alpha_pv[no_joints] = { 14.0, 14.0, 14.0 };
    constexpr double kd_pv[no_joints] = { 0.35, 0.35, 0.35 };

				static double sigma_pv[no_joints] = { 0.0, 0.0, 0.0 };

    constexpr double dote_pos[no_joints] = { 0.0,0.0,0.0 };
    constexpr double e_pos[no_joints] = { 0.0,0.0,0.0 };
				double dqd[no_joints] = { 0.0,0.0,0.0 };
				double dot_qr_pv[no_joints] = { 0.0, 0.0, 0.0 };
				double dot_sigma_pv[no_joints] = { 0.0, 0.0, 0.0 };
				double s_pv[no_joints] = { 0.0, 0.0, 0.0 };
    constexpr double sd_pv[no_joints] = { 0.0, 0.0, 0.0 };
				double sr_pv[no_joints] = { 0.0, 0.0, 0.0 };
				double sq_pv[no_joints] = { 0.0, 0.0, 0.0 };

				if (i_c_smc) {
								ti = timeGetTime();
				}

				const double t = (timeGetTime() - ti) / 1000.0;

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
								sigma_pv[i] += dot_sigma_pv[i] * sample_time;
								dot_qr_pv[i] = dqd[i] - alpha_pv[i] * e_pos[i] + sd_pv[i] - gamma_pv[i] * sigma_pv[i];
								dot_sigma_pv[i] = HelperFunctions::Sign(sq_pv[i]);
								s_pv[i] = dote_pos[i] + alpha_pv[i] * e_pos[i];
								sq_pv[i] = s_pv[i];
								sr_pv[i] = sq_pv[i] + gamma_pv[i] * sigma_pv[i];

								tau[i] = -kd_pv[i] * sr_pv[i];
				}

				std::vector<std::vector<double>> tau_graph_data = { tau, std::vector<double>(25) };

				return tau_graph_data;
}

std::vector<std::vector<double>> controllers::nl_controller(const double pi, const double sample_time,
				const bool i_c_smc, double *qm, static double ti) {
				static double qm_1[no_joints] = { 0, 90.0 * pi / 180.0, -90.0 * pi / 180.0 };
				static double sigma[no_joints] = { 0.0,0.0,0.0 };
				constexpr double alpha = 9.0 / 11.0;


				double dote_pos[no_joints] = { 0.0,0.0,0.0 };
				double e_pos[no_joints] = { 0.0,0.0,0.0 };
				double sq[no_joints] = { 0.0,0.0,0.0 };
				double dot_qr[no_joints] = { 0.0,0.0,0.0 };
				double dqm[no_joints] = { 0, 0, 0 };
				double s[no_joints] = { 0.0,0.0,0.0 };
				double qd[no_joints] = { 0.0,0.0,0.0 };
				double dqd[no_joints] = { 0.0,0.0,0.0 };
				double dot_sigma[no_joints] = { 0.0,0.0,0.0 };
				double vel[no_joints] = { 0.0, 0.0, 0.0 };

				constexpr double k1[no_joints] = { 11, 16, 12 };
				constexpr double k2[no_joints] = { 0.05, 0.08, 0.06 };
				constexpr double k3[no_joints] = { 0.05, 0.05, 0.05 };
				constexpr double k4[no_joints] = { 0.16, 0.28, 0.26 };

				if (i_c_smc) {
								ti = timeGetTime();
								std::copy_n(qm, no_joints, std::begin(qm_1));
				}

				const double t = (timeGetTime() - ti) / 1000.0;

				vel[0] = HelperFunctions::RED1(qm[0], sample_time);
				vel[1] = HelperFunctions::RED2(qm[1], sample_time);
				vel[2] = HelperFunctions::RED3(qm[2], sample_time);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
								dqm[i] = (qm[i] - qm_1[i]) * sample_time;
								e_pos[i] = qm[i] - qd[i];
								dote_pos[i] = vel[i] - dqd[i];
								sq[i] = dote_pos[i] + k1[i] * HelperFunctions::Sign(e_pos[i]) * (pow((abs(e_pos[i])), alpha));
								dot_qr[i] = dqd[i] - k1[i] * HelperFunctions::Sign(e_pos[i]) * pow((abs(e_pos[i])), alpha) - k2[i] * sigma[i];
								s[i] = dqm[i] - dot_qr[i];
								dot_sigma[i] = k3[i] * sq[i] + HelperFunctions::Sign(sq[i]);
								sigma[i] += dot_sigma[i] * sample_time;

								tau[i] = k4[i] * tanh(s[i]);
				}

				std::vector<std::vector<double>> tau_graph_data = { tau,
								graph_trajectory(t, pi, qm, dote_pos, e_pos, qd, dqd, vel, dot_qr) };

				return tau_graph_data;
}

std::vector<double> controllers::graph_trajectory(const double t, const double pi, const double* qm, const double* dote_pos, 
				const double* e_pos, const double* qd, const double* dqd, const double* vel, const double* dot_qr)
{
				std::vector<double> graph_data(25);

				graph_data[0] = t;
				graph_data[1] = qm[0] * 180.0 / pi;
				graph_data[2] = qm[1] * 180.0 / pi;
				graph_data[3] = qm[2] * 180.0 / pi;
				graph_data[4] = dote_pos[0] * 180.0 / pi;
				graph_data[5] = dote_pos[1] * 180.0 / pi;
				graph_data[6] = dote_pos[2] * 180.0 / pi;
				graph_data[7] = e_pos[0] * 180.0 / pi;
				graph_data[8] = e_pos[1] * 180.0 / pi;
				graph_data[9] = e_pos[2] * 180.0 / pi;
				graph_data[10] = qd[0] * 180.0 / pi;
				graph_data[11] = qd[1] * 180.0 / pi;
				graph_data[12] = qd[2] * 180.0 / pi;
				graph_data[13] = abs(tau[0]);
				graph_data[14] = abs(tau[1]);
				graph_data[15] = abs(tau[2]);
				graph_data[16] = dqd[0] * 180.0 / pi;
				graph_data[17] = dqd[1] * 180.0 / pi;
				graph_data[18] = dqd[2] * 180.0 / pi;
				graph_data[19] = vel[0] * 180.0 / pi;
				graph_data[20] = vel[1] * 180.0 / pi;
				graph_data[21] = vel[2] * 180.0 / pi;
				graph_data[22] = dot_qr[0];
				graph_data[23] = dot_qr[1];
				graph_data[24] = dot_qr[2];

				return graph_data;
}
