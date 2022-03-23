#pragma once

#include "vector"
#include "complex"
#include "memory"
#include "gsl/gsl_poly.h"
#include "boost/shared_ptr.hpp"

namespace ctrl {
/** @brief Polynomial Solver
 *
 * Take a polynomial defined as
 *   f(x) = a_n x^n + a_(n-1) x^(n-1) + ... + a_1 x + a_0
 * std::vector<double>{-1, 0, 0, 0, 1} Represents f(x) = x^4 - 1
 * Class #PolynomialSolver finds the roots of that polynomial
 */
    class PolynomialSolver {
    private:

        /** @brief Coefficient vector
         *
         * nth variable of the vector describes the coefficient for nth degree
         */
        std::vector<double> m_coeff;

    public:

        /** @brief Trivial constructor
         *
         */
        PolynomialSolver() = default;

        /** @brief Constructor that sets coefficients
         *
         * @param coeff
         */
        explicit PolynomialSolver(std::vector<double> coeff);

        /** @brief Trivial setter for coefficients
         *
         * @param coeff
         */
        void set_coeff(decltype(m_coeff) coeff);

        /** @brief Trivial getters for coefficients
         *
         * @return
         */
        auto get_coeff() -> decltype(m_coeff);

        /** @brief Solves the polynomial
         *
         * @param roots Roots of the polynomial stored in the reference of parameter
         * @return If operation is successful or not
         */
        bool solve(std::vector<std::complex<double>> &roots);

        /** @brief Solves the polynomial for different Y value
         *
         * @param roots Roots of the polynomial stored in the reference of parameter
         * @param y Requested y value
         * @return
         */
        bool solve_for_y(std::vector<std::complex<double>> &roots, double y);

        //! @brief Trivial smart pointer for polynomial solver
        typedef boost::shared_ptr<PolynomialSolver> Ptr;

    };
}