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

#include "polynomial_solver.h"
#include "utility"
#include "cstring"

using namespace ctrl;

PolynomialSolver::PolynomialSolver(std::vector<double> coeff) : m_coeff(std::move(coeff)){

}

auto PolynomialSolver::get_coeff() -> decltype(this->m_coeff) {
    return m_coeff;
}

void PolynomialSolver::set_coeff(decltype(m_coeff) coeff) {
    m_coeff = std::move(coeff);
}

bool PolynomialSolver::solve_for_y(std::vector<std::complex<double>> &roots, double y) {

    if(std::isnan(y)){
        return false;
    }

    std::vector<double> coeff = m_coeff;
    coeff.front() -= y;
    {
        auto n = (int)coeff.size();
        auto n_z = (n-1) *2;

        auto a = new double[n];
        auto z = new double[n_z];

        memcpy(a, coeff.data(), sizeof(double) * coeff.size());

        gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc (n);
        gsl_poly_complex_solve (a, n, w, z);
        gsl_poly_complex_workspace_free (w);

        for(int i = 0; i < n_z * 2; i += 2) {
            roots.emplace_back(std::complex<double>(z[i],z[i+1]));
        }

        delete[] a;
        delete[] z;
    }

    return true;
}

bool PolynomialSolver::solve(std::vector<std::complex<double>>& roots) {
    return solve_for_y(roots, 0);
}

