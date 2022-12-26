/*
    This file is part of MVP-Control program.

    MVP-Control is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Control is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Control.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "polynomial_solver.hpp"
#include "gsl/gsl_poly.h"
#include "cstring"

using namespace mvp;

bool PolynomialSolver::solve(std::vector<double> coefficients,
                             std::vector<std::complex<double>> *roots,
                             double* solution,
                             double y) {

    // Check if the Y value is a "number"
    if(std::isnan(y)){
        return false;
    }

    bool ifreal = false;

    // Move the polynomial by "y"
    coefficients.front() -= y;

    // apply it to GSL poly solver
    auto n = (int)coefficients.size();
    auto n_z = (n-1) *2;
    auto a = new double[n];
    auto z = new double[n_z];
    std::memcpy(a, coefficients.data(), sizeof(double) * coefficients.size());

    gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc(n);
    gsl_poly_complex_solve(a, n, w, z);
    gsl_poly_complex_workspace_free(w);

    // Save the roots to vector and find the real solution
    for(int i = 0; i < n_z * 2; i += 2) {
        if(roots != nullptr) {
            roots->emplace_back(std::complex<double>(z[i], z[i + 1]));
        }

        if(solution != nullptr) {
            if(z[i + 1] == 0) {
                *solution = z[i];
                ifreal = true;
            }
        }

    }

    delete[] a;
    delete[] z;

    return ifreal;
}

