
#include <rach_ik/constraint.h>

double constraint_lin(unsigned n, const double *x, double *grad, void *data)
{
    constraint_data *d = (constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
 }