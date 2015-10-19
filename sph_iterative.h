#ifndef __SPH_ITERATIVE_H__
#define __SPH_ITERATIVE_H__
#include "Calculus.h"	
#include <iostream>

using namespace std;


class SPH_Iterative : public Calculus
{
private:

	double compute_density(shared_ptr<Particle> p);
	double compute_pressure(shared_ptr<Particle> p);

	void non_recurrent_part();
	void recurrent_part();
	double nu;
	Kernel * w_kernel;
	double delta_t;
	vector<shared_ptr<Particle>> modified_particles;
	void modify_grid();
	double rho_zero;
	double k;

public:
	SPH_Iterative(Grid *g, double viscosity, double nu, double smoothingDistance, double delta_t, double rho_zero);
	~SPH_Iterative();
	void proceed();
};
#endif