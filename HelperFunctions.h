#include <cmath>

class HelperFunctions {
public:
				static double Sign(double f);
				static double Levantq1(double f, const double SAMPLE_TIME);
				static double Levantq2(double f, const double SAMPLE_TIME);
				static double Levantq3(double f, const double SAMPLE_TIME);
				static double RED1(double f, const double SAMPLE_TIME);
				static double RED2(double f, const double SAMPLE_TIME);
				static double RED3(double f, const double SAMPLE_TIME);
};