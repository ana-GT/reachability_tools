#pragma once

typedef struct {
    double a, b;
} constraint_data;

double constraint_lin(unsigned n, const double *x, double *grad, void *data);
