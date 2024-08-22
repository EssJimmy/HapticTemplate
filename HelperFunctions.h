// contains helper functions for trajectory calculations
// it places them in HelperFunctions.cpp instead of in
// the main code

class HelperFunctions {
public:
				static double Sign(double f);
				static double Levantq1(double pos1, const double SAMPLE_TIME);
				static double Levantq2(double pos2, const double SAMPLE_TIME);
				static double Levantq3(double pos3, const double SAMPLE_TIME);
				static double RED1(double posq1, const double SAMPLE_TIME);
				static double RED2(double posq2, const double SAMPLE_TIME);
				static double RED3(double posq3, const double SAMPLE_TIME);
};