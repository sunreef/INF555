#ifndef __CALCULUS_H_
#define __CALCULUS_H_



class Calculus
{
private:
	double viscosity;
	int numberOfParticles;
	Grid *grid;

	static Vect calcule_gradient_pressure(Particle *p, Kernel *w_kernel)
	{
		Vect result = 0 ;
		double current_density = p->rho;

		for(int j = 0 ; j < p->neighbours.size(); j++)
		{
			double temp_density = p->neighbours[j]->rho;
			double temp = p->pressure / (current_density * current_density) + p->neighbours[j]->pressure / ( temp_density * temp_density);

			temp*=this->neighbours[j]->w;
			Vect temp_vec = w_kernel->grad(p->pos, p->neighbours[j]->pos);

			result.x += temp * temp_vec.x;
			result.y += temp * temp_vec.y;
			result.z += temp * temp_vec.z;
		}

		result.x *= current_density;
		result.y *= current_density;
		result.z *= current_density;
		return result;
	}
	static double calcule_laplace_speed_coordinate(int index, Particle *p, Kernel *w_kernel)
	{
		double result = 0;
		double current_density = p->rho;
		for(int j = 0 ; j < p->neighbours.size(); j++)
		{
			double temp_density = p->neighbours[j]->rho;
			double A_ij = p->speed[index] - p->neighbours[j]->speed[index];
			Vect x_ij = p->pos - p->neighbours[j]->pos;

			double temp = p->neighbours[j]->w / (temp_density) * A_ij;
			
			double scalar_product = w_kernel->grad(p->pos, p->neighbours[j]->pos) * x_ij;
			temp *= scalar_product;
			double denominator = x_ij * x_ij;
			denominator += 0.001 * w_kernel->h * w_kernel->h;
			temp/=denominator;
			result+= temp;
		}

		return 2 * result;

	}
	static Vect calcule_laplace_speed(Particle *p, Kernel *w_kernel)
	{

		  Vect result(0,0,0);
		  for(int i = 0 ; i < 3; i++)
		  {
		  	result[i] = calcule_laplace_speed_coordinate(i, p, w_kernel);
		  }

		  return result;

	}
public:
	virtual void proceed();
};

#endif