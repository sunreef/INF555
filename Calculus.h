#ifndef __CALCULUS_H_
#define __CALCULUS_H_


class Calculus {
private:
    double viscosity;
    int numberOfParticles;
    Grid *grid;

    static Vect computeGradientPressure(shared_ptr<Particle> p, Kernel *w_kernel) {
        Vect result(0, 0, 0);
        double current_density = p->rho;

        for (shared_ptr<Particle> neighbour: p->neighbours) {
            double temp_density = neighbour->rho;
            double temp = p->pressure / (current_density * current_density) +
                          neighbour->pressure / (temp_density * temp_density);

            temp *= neighbour->w;
            Vect temp_vec = w_kernel->grad(p->pos, neighbour->pos);

            result.x += temp * temp_vec.x;
            result.y += temp * temp_vec.y;
            result.z += temp * temp_vec.z;
        }

        result.x *= current_density;
        result.y *= current_density;
        result.z *= current_density;
        return result;
    }

    static double computeLaplaceSeedCoordinate(int index, shared_ptr<Particle> p, Kernel *w_kernel) {
        double result = 0;
        double current_density = p->rho;
        for (shared_ptr<Particle> neighbour: p->neighbours) {
            double temp_density = neighbour->rho;
            double A_ij = p->speed[index] - neighbour->speed[index];
            Vect x_ij = p->pos - neighbour->pos;

            double temp = neighbour->w / (temp_density) * A_ij;

            double scalar_product = w_kernel->grad(p->pos, neighbour->pos) * x_ij;
            temp *= scalar_product;
            double denominator = x_ij * x_ij;
            denominator += 0.01 * w_kernel->getSmoothingDistance() * w_kernel->getSmoothingDistance();
            temp /= denominator;
            result += temp;
        }

        return 2 * result;

    }

    static Vect computeLaplaceSeed(shared_ptr<Particle> p, Kernel *w_kernel) {

        Vect result(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            result[i] = computeLaplaceSeedCoordinate(i, p, w_kernel);
        }

        return result;

    }

public:
    virtual void proceed();
};

#endif