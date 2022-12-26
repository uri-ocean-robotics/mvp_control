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

#pragma once

#include "vector"
#include "complex"

namespace mvp {
    /** @brief Polynomial Solver
     *
     * Take a polynomial defined as
     *   f(x) = a_n x^n + a_(n-1) x^(n-1) + ... + a_1 x + a_0
     * std::vector<double>{-1, 0, 0, 0, 1} Represents f(x) = x^4 - 1
     * Class #PolynomialSolver finds the roots of that polynomial
     */
    class PolynomialSolver {
    public:

        /** @brief Trivial constructor
         *
         */
        PolynomialSolver() = default;

        static bool solve(std::vector<double> coefficients, std::vector<std::complex<double>>* roots, double* solution, double y = 0);

    };
}