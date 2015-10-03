#include "sph_iterative.h"	

using namespace std;

double const g_value = 9.8;

double SPH_Iterative::compute_density(Particle *p)
{
	double result = p->rho / this->rho_zero;

	result = 
}
double SPH_Iterative::compute_pressure(Particle *p)
{

}




void SPH_Iterative::non_recurrent_part()
{
	Vect g(0,0,-g_value0);

	for(int i = 0 ; i < this->g->particles.size(); i++)
	{
		Particle p = this->g->particles[i];
		double temp_coeff = p.w * this->viscosity;
		p.force_viscosity = Calculus.calcule_laplace_speed(&p, this->w_kernel);
		p.force_viscosity *= temp_coeff;

		p.force_other = g * p.w;
		Vect temp = p.force_viscosity + p.force_other;
		temp /= p.w;
		temp *= this->delta_t;

		p.speed = p.speed + temp;
		temp = p.speed * delta_t;

		p.pos = p_pos + temp;
		this->modified_particles[i] = p;
	}
}
void SPH_Iterative::recurrent_part()
{
	double density_error = 999999;
	do
	{
		for(int i = 0 ; i < this->g->particles.size(); i++)
		{
			Particle p = this->modified_particles[i];
			this->modified_particles[i].rho = compute_density(&p);
			this->modified_particles[i].pressure = compute_pressure(&p);
		}

		double density_error = calcule_density_error();

		for(int i = 0 ; i < this->g->particles.size(); i++)
		{
			Particle p = this->modified_particles[i];
			Vect temp = Calculus.calcule_gradient_pressure(&p, this->w_kernel);
			temp *= ( - p.w / p.rho);
			this->modified_particles[i].force_pressure = temp;

			Vect temp_force = this->modified_particles[i].force_pressure / p.w;
			temp_force *= this->delta_t;

			this->modified_particles[i].speed += temp_force;

			temp_force *= this->delta_t;
			thos->modified_particles[i].pos += temp_force;
		}
	}
	while(density_error > this->nu);
}


SPH_Iterative::SPH_Iterative(Grid *g, double viscosity, double num,double smoothingDistance, double delta_t, double rho_zero, double k)
{
	this->g = g;
	this->viscosity = viscosity;
	this->nu = nu;
	this->numberOfParticles = g->numberOfParticles;
	this->w_kernel = new Kernel(smoothingDistance);
	this->delta_t = delta_t;
	for(int i = 0 ; i < this->numberOfParticles; i++)
	{
		this->modified_particles.push_back(Particle(0, 0, 0, 0, 0));
	}
	this->rho_zero = rho_zero;
	this->k = k;
}
SPH_Iterative::~SPH_Iterative()
{
	delete g;
}
void SPH_Iterative::modify_grid()
{
	for(int i = 0 ; i < this->numberOfParticles; i++)
	{
		this->g->particles[i].copy(this->modified_particles[i]);
	}

	this->grid->update();
}
void SPH_Iterative::proceed()
{

	this->non_recurrent_part();
	this->recurrent_part();

}
