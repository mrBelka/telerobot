#include <iostream>
#include <cmath>

#define PI 3.141592653589793238


struct FORWARD_DATA{
		double WFL;
		double WFR;
		double WRL;
		double WRR;
		FORWARD_DATA() = default;
};


struct INVERSE_DATA {
	double VX;
	double VY;
	double WZ;

	INVERSE_DATA() = default;
};


struct OdometryClass {
	double r, lx, ly, rot;
	double trans_matrix[3][3];
	double vector[2];
	double posx = 0.0, posy = 0.0;
	OdometryClass(
		double r,
		double lx,
		double ly,
		double rot = 90.0
		): r(r), lx(lx), ly(ly), rot(rot) {}

	void Update(double dt, FORWARD_DATA data) {
		INVERSE_DATA mv = GetOdometryFORW(data);

		std::cout << mv.VX << " " << mv.VY << " " << mv.WZ << std::endl;

		vector[0] = mv.VX;
		vector[1] = mv.VY;

		posx = (std::cos(mv.WZ * (PI / 180))*vector[0]*dt - std::sin(mv.WZ * (PI / 180))*vector[1]*dt + posx);
		posy = (std::sin(mv.WZ * (PI / 180))*vector[0]*dt + std::cos(mv.WZ * (PI / 180))*vector[1]*dt + posy);
	}

	void Update(double dt, INVERSE_DATA mv) {
		vector[0] = mv.VX;
		vector[1] = mv.VY;

		posx = (std::cos(mv.WZ)*vector[0]*dt - std::sin(mv.WZ)*vector[1]*dt + posx);
		posy = (std::sin(mv.WZ)*vector[0]*dt + std::cos(mv.WZ)*vector[1]*dt + posy);
	}


	INVERSE_DATA GetOdometryFORW(FORWARD_DATA data) {
		return {
			(data.WFL + data.WFR + data.WRL + data.WRR) * (r/4),
			(-data.WFL + data.WFR + data.WRL - data.WRR) * (r/4),
			(-data.WFL + data.WFR - data.WRL + data.WRR) * (r/4) * (1/(lx + ly))
		};
	}

	FORWARD_DATA GetOdometryINV(INVERSE_DATA data) {
		return {
			(1/r) * (data.VX - data.VY - (lx + ly)*data.WZ),
			(1/r) * (data.VX + data.VY + (lx + ly)*data.WZ),
			(1/r) * (data.VX + data.VY - (lx + ly)*data.WZ),
			(1/r) * (data.VX - data.VY + (lx + ly)*data.WZ)
		};
	}
};

