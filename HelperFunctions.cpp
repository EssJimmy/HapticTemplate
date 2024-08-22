// HelperFunctions.h implementation

#include "pch.h"
#include "HelperFunctions.h"
#include <cmath>

double HelperFunctions::Sign(double f) {
				double s = 0;
				if (f > 0) s = 1;
				else if (f < 0) s = -1;

				return s;
}

double HelperFunctions::RED1(double posq1, const double SAMPLE_TIME) {
				static double u = 0, u1 = 0, dx = 0, x = 0, du = 0;
				double aux1 = abs(x - posq1), aux2 = 0.5, s = Sign(x - posq1);

				u1 += du * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;
				u = u1 - 5.5 * (pow(aux1, aux2)) * s;
				dx = u;
				du = -0.7 * s;

				return u;
}

double HelperFunctions::RED2(double posq2, const double SAMPLE_TIME) {
				static double u = 0, u1 = 0, x = 1.57;
				double dx = u, s = Sign(x - posq2);

				u = u1 - 1.5 * (pow(abs(x - posq2), 0.5)) * s;
				u1 += (-0.8 * s) * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;

				return u;
}

double HelperFunctions::RED3(double posq3, const double SAMPLE_TIME) {
				static double u = 0, u1 = 0, x = -1.57;
				double dx = u, s = Sign(x - posq3);

				u = u1 - 1.5 * (pow(abs(x - posq3), 0.5)) * s;
				u1 += (-0.8 * s) * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;

				return u;
}

double HelperFunctions::Levantq1(double pos1, const double SAMPLE_TIME) {
				static double z0 = 0.0;
				static double z1 = 0.0;
				static double z2 = 0.0;

				const double L = 3;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 0.33)) * (pow(abs(z0 - pos1), 0.66)) * (Sign(z0 - pos1)) + z1;
				v1 = -(1.5) * (pow(L, 0.5)) * (pow(abs(z1 - pos1), 0.5)) * (Sign(z1 - v0)) + z2;
				dz2 = -2.0 * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}

double HelperFunctions::Levantq2(double pos2, const double SAMPLE_TIME) {
				const double L = 3.0;

				static double z0 = 1.65;
				static double z1 = 0.0;
				static double z2 = 0.0;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 1 / 3)) * (pow((abs(z0 - pos2)), (2 / 3))) * (Sign(z0 - pos2)) + z1;
				v1 = -(1.5) * (pow(L, 1 / 2)) * (pow((abs(z1 - pos2)), (1 / 2))) * (Sign(z1 - v0)) + z2;
				dz2 = -(2.0) * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}

double HelperFunctions::Levantq3(double pos3, const double SAMPLE_TIME) {
				const double L = 3.0;

				static double z0 = -1.74;
				static double z1 = 0.0;
				static double z2 = 0.0;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 1 / 3)) * (pow((abs(z0 - pos3)), (2 / 3))) * (Sign(z0 - pos3)) + z1;
				v1 = -(1.5) * (pow(L, 1 / 2)) * (pow((abs(z1 - pos3)), (1 / 2))) * (Sign(z1 - v0)) + z2;
				dz2 = -(2.0) * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}