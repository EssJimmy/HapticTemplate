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

				constexpr double kp[no_joints] = { 1.1,1.2,2.0 };
				constexpr double ki[no_joints] = { 0.2,0.2,0.2 };
				constexpr double kd[no_joints] = { 0.1,0.2,0.2 }; // dont touch this

				double dot_delta_pos[no_joints] = { 0.0,0.0,0.0 };
				double delta_pos[no_joints] = { 0.0,0.0,0.0 };
				double qd[no_joints] = { 0.0,0.0,0.0 };
				double dot_qd[no_joints] = { 0.0,0.0,0.0 };
				double delta_pos_i[no_joints] = { 0.0, 0.0, 0.0 };
				double dot_qm[no_joints] = { 0.0, 0.0, 0.0 };

				constexpr double k1[no_joints] = { 11, 16, 12 };

				if (i_c_smc) {
								ti = timeGetTime();
				}

				const double t = (timeGetTime() - ti) / 1000.0;

    // Abbreviated Central Derivative approximation
				dot_qm[0] = HelperFunctions::ForwardsDerivative(qm_1[0], qm[0], sample_time);
				dot_qm[1] = HelperFunctions::ForwardsDerivative(qm_1[1], qm[1], sample_time);
				dot_qm[2] = HelperFunctions::ForwardsDerivative(qm_1[2], qm[2], sample_time);

				// dot qm / current position derivative
				//dot_qm[0] = HelperFunctions::RED1(qm[0], sample_time);
				//dot_qm[1] = HelperFunctions::RED2(qm[1], sample_time);
				//dot_qm[2] = HelperFunctions::RED3(qm[2], sample_time);

				// desired position
				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				// desired velocity
				dot_qd[0] = -0.12 * sin(3 * t);
				dot_qd[1] = -0.2 * sin(t);
				dot_qd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
        delta_pos[i] = qm[i] - qd[i]; // current position error
        dot_delta_pos[i] = dot_qm[i] - dot_qd[i]; // current velocity error
        delta_pos_i[i] += delta_pos[i] * sample_time; // integral of the error

								tau[i] = -kp[i] * delta_pos[i] - ki[i] * delta_pos_i[i] - kd[i] * dot_delta_pos[i]; //pid standard shenanigans
				}

				std::vector<std::vector<double>> tau_graph_data = { tau,
				    graph_trajectory(t, pi, qm, dot_delta_pos, delta_pos, qd, dot_qd, dot_qm, dot_qr) };

				std::copy_n(qm, no_joints, std::begin(qm_1));

				return tau_graph_data;
}

std::vector<std::vector<double>> controllers::parra_vega_controller(const double pi, const double sample_time, const bool i_c_smc,
				static double ti, const double* qm) {
    constexpr double gamma[no_joints] = { 0.4, 0.4, 0.4 };
    constexpr double alpha[no_joints] = { 4.0, 4.0, 4.0 };
    constexpr double kd[no_joints] = { 0.35, 0.35, 0.35 };

				static double sigma[no_joints] = { 0.0, 0.0, 0.0 };
    static double qm_1[no_joints] = { 0, 90.0 * pi / 180.0, -90.0 * pi / 180.0 };

    double dot_delta_pos[no_joints] = { 0.0,0.0,0.0 };
    double delta_pos[no_joints] = { 0.0,0.0,0.0 };
    double dot_qm[no_joints] = { 0.0, 0.0, 0.0 };
    double qd[no_joints] = { 0.0,0.0,0.0 };
				double dot_qd[no_joints] = { 0.0,0.0,0.0 };
				double dot_qr[no_joints] = { 0.0, 0.0, 0.0 };
				double dot_sigma[no_joints] = { 0.0, 0.0, 0.0 };
				double s[no_joints] = { 0.0, 0.0, 0.0 };
    constexpr double sd[no_joints] = { 0.0, 0.0, 0.0 };
				double sr[no_joints] = { 0.0, 0.0, 0.0 };
				double sq[no_joints] = { 0.0, 0.0, 0.0 };

				if (i_c_smc) {
								ti = timeGetTime();
								std::copy_n(qm, no_joints, std::begin(qm_1));
				}

				const double t = (timeGetTime() - ti) / 1000.0;

				// current position velocity
				dot_qm[0] = HelperFunctions::RED1(qm[0], sample_time);
				dot_qm[1] = HelperFunctions::RED2(qm[1], sample_time);
				dot_qm[2] = HelperFunctions::RED3(qm[2], sample_time);

				// desired trajectory
    qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

    // desired velocity
				dot_qd[0] = -0.12 * sin(3 * t);
				dot_qd[1] = -0.2 * sin(t);
				dot_qd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
        delta_pos[i] = qm[i] - qd[i]; // current position error
        dot_delta_pos[i] = dot_qm[i] - dot_qd[i]; // current velocity error
        dot_sigma[i] = HelperFunctions::Sign(sq[i]); // sign of the error
								sigma[i] += dot_sigma[i] * sample_time; // error in time
				    dot_qr[i] = dot_qd[i] - alpha[i]*delta_pos[i] + sd[i] - gamma[i] * sigma[i]; // nominal reference for nl pid
								
								s[i] = dot_delta_pos[i] + alpha[i] * delta_pos[i];
								sq[i] = s[i];
								sr[i] = sq[i] + gamma[i] * sigma[i];

								tau[i] = -kd[i] * sr[i];
				}

				std::vector<std::vector<double>> tau_graph_data = { tau,
				    graph_trajectory(t, pi, qm, dot_delta_pos, delta_pos, qd, dot_qd, dot_qm, dot_qr)};

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
				double dot_qm[no_joints] = { 0.0, 0.0, 0.0 };

				constexpr double k1[no_joints] = { 11, 16, 12 };
				constexpr double k2[no_joints] = { 0.05, 0.08, 0.06 };
				constexpr double k3[no_joints] = { 0.05, 0.05, 0.05 };
				constexpr double k4[no_joints] = { 0.16, 0.28, 0.26 };

				if (i_c_smc) {
								ti = timeGetTime();
				}

				const double t = (timeGetTime() - ti) / 1000.0;

				dot_qm[0] = HelperFunctions::RED1(qm[0], sample_time);
				dot_qm[1] = HelperFunctions::RED2(qm[1], sample_time);
				dot_qm[2] = HelperFunctions::RED3(qm[2], sample_time);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < no_joints; i++) {
								dqm[i] = (qm[i] - qm_1[i]) * sample_time;
								e_pos[i] = qm[i] - qd[i];
								dote_pos[i] = dot_qm[i] - dqd[i];
								sq[i] = dote_pos[i] + k1[i] * HelperFunctions::Sign(e_pos[i]) * (pow((abs(e_pos[i])), alpha));
								dot_qr[i] = dqd[i] - k1[i] * HelperFunctions::Sign(e_pos[i]) * pow((abs(e_pos[i])), alpha) - k2[i] * sigma[i];
								s[i] = dqm[i] - dot_qr[i];
								dot_sigma[i] = k3[i] * sq[i] + HelperFunctions::Sign(sq[i]);
								sigma[i] += dot_sigma[i] * sample_time;

								tau[i] = k4[i] * tanh(s[i]);
				}

				std::vector<std::vector<double>> tau_graph_data = { tau,
								graph_trajectory(t, pi, qm, dote_pos, e_pos, qd, dqd, dot_qm, dot_qr) };

				std::copy_n(qm, no_joints, std::begin(qm_1));

				return tau_graph_data;
}
std::vector<std::vector<double>> controllers::adaptive_controller(const double pi, const double sample_time, 
				const bool i_c_smc, double* qm, static double ti) {
    static double qm_1[no_joints] = { 0, 90.0 * pi / 180.0, -90.0 * pi / 180.0 };
				
				const double sigma_c = 10;
				const double phi[8] = { 0.00251729 ,0.00108246 ,0.00137408 ,0.00076823 ,0.03526735 ,0.00744473 ,0.00449158 ,0.00534505 };
				const double h_hat[no_joints][no_joints] = { {pow(cos(qm[1]), 2) * phi[0] + cos(qm[1])*cos(qm[1]+qm[2]) * phi[1] * pow(sin(qm[1] + qm[2]), 2) * phi[2], 0, 0},
								{0, phi[0] + 2 * cos(qm[2]) * phi[1] + phi[2], cos(qm[2]) * phi[2] + phi[2]},
								{0, cos(qm[2]) * phi[2] + phi[2], phi[2]}
				};

				const double alpha[no_joints] = { sigma_c, sigma_c, sigma_c };
    const double dot_qm[no_joints] = { HelperFunctions::ForwardsDerivative(qm_1[0], qm[0], sample_time),
				HelperFunctions::ForwardsDerivative(qm_1[1], qm[1], sample_time), HelperFunctions::ForwardsDerivative(qm_1[2], qm[2], sample_time) };
				double k_d[no_joints][no_joints];

				for (int i = 0; i < no_joints; i++) {
        for (int j = 0; j < no_joints; j++) {
            k_d[i][j] = h_hat[i][j] * sigma_c;
        }
    }

				if (i_c_smc) {
								ti = timeGetTime();
				}

				const double t = (timeGetTime() - ti) / 1000.0;
				const double dot_qd[no_joints] = { -0.12 * sin(3 * t), -0.2 * sin(t), 0.4 * sin(2 * t) };
				const double qd[no_joints] = {-0.4 + 0.4 * cos(3 * t), 1.37 + 0.2 * cos(t), -1.37 - 0.2 * cos(2 * t)};
				double delta_qm[no_joints] = { 0.0, 0.0, 0.0 };
				double delta_dot_qm[no_joints] = { 0.0, 0.0, 0.0 };
    double kd_s[no_joints] = { 0.0, 0.0, 0.0 };
    double s[no_joints] = { 0.0, 0.0, 0.0 };

				for (int i = 0; i < no_joints; i++) {
        delta_qm[i] = qm[i] - qd[i];
								delta_dot_qm[i] = dot_qm[i] - qd[i];
        s[i] = delta_dot_qm[i] + alpha[i] * delta_qm[i];
        kd_s[i] = (k_d[i][0] + k_d[i][1] + k_d[i][2]) * delta_dot_qm[i] + k_d[i][i]*alpha[i]*delta_qm[i];
				}

				
    
    std::copy_n(qm, no_joints, std::begin(qm_1));
				std::vector<std::vector<double>> adaptive_tau_graph_data;
    return adaptive_tau_graph_data;
}

std::vector<double> controllers::graph_trajectory(const double t, const double pi, const double* qm, const double* dot_delta_pos, 
				const double* delta_pos, const double* qd, const double* dot_qd, const double* dot_qm, const double* dot_qr)
{
				std::vector<double> graph_data(25);

				graph_data[0] = t;
				graph_data[1] = qm[0] * 180.0 / pi;
				graph_data[2] = qm[1] * 180.0 / pi;
				graph_data[3] = qm[2] * 180.0 / pi;
				graph_data[4] = dot_delta_pos[0] * 180.0 / pi;
				graph_data[5] = dot_delta_pos[1] * 180.0 / pi;
				graph_data[6] = dot_delta_pos[2] * 180.0 / pi;
				graph_data[7] = delta_pos[0] * 180.0 / pi;
				graph_data[8] = delta_pos[1] * 180.0 / pi;
				graph_data[9] = delta_pos[2] * 180.0 / pi;
				graph_data[10] = qd[0] * 180.0 / pi;
				graph_data[11] = qd[1] * 180.0 / pi;
				graph_data[12] = qd[2] * 180.0 / pi;
				graph_data[13] = abs(tau[0]);
				graph_data[14] = abs(tau[1]);
				graph_data[15] = abs(tau[2]);
				graph_data[16] = dot_qd[0] * 180.0 / pi;
				graph_data[17] = dot_qd[1] * 180.0 / pi;
				graph_data[18] = dot_qd[2] * 180.0 / pi;
				graph_data[19] = dot_qm[0] * 180.0 / pi;
				graph_data[20] = dot_qm[1] * 180.0 / pi;
				graph_data[21] = dot_qm[2] * 180.0 / pi;
				graph_data[22] = dot_qr[0];
				graph_data[23] = dot_qr[1];
				graph_data[24] = dot_qr[2];

				return graph_data;
}
