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
	double r, lx, ly;
	double trans_matrix[3][3];
	double vector[2];
	double posx = 0.0, posy = 0.0, rot = 0;
	INVERSE_DATA mv_after_update;
	OdometryClass(
		double r,
		double lx,
		double ly
		): r(r), lx(lx), ly(ly) {}

	void Update(FORWARD_DATA data) {
//		std::cout << data.WFL << " " << data.WFR << " " << data.WRL << " " << data.WRR << std::endl;
		mv_after_update = GetOdometryFORW(data);
		INVERSE_DATA& mv = mv_after_update;

//		std::cout << mv.VX << " " << mv.VY << " " << mv.WZ << std::endl;

		vector[0] = mv.VX;
		vector[1] = mv.VY;

		rot += mv.WZ;
		posx = (std::cos(rot)*vector[0] - std::sin(rot)*vector[1] + posx);
		posy = (std::sin(rot)*vector[0] + std::cos(rot)*vector[1] + posy);
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

