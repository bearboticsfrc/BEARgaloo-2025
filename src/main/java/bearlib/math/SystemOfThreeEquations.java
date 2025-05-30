package bearlib.math;

import Jama.Matrix;

/**
 * Represents a system of three linear equations with three variables (x, y, z) of the form:
 *
 * <pre>
 * a1*x + b1*y + c1*z = d1
 * a2*x + b2*y + c2*z = d2
 * a3*x + b3*y + c3*z = d3
 * </pre>
 *
 * This class provides functionality to solve the system of equations and obtain the values of x, y,
 * and z. It uses the Jama library for matrix operations.
 */
public class SystemOfThreeEquations {

  /** The matrix representing the coefficients of the variables in the system of equations. */
  private Matrix coefficients;

  /** The matrix representing the constants (right-hand side values) of the system of equations. */
  private Matrix constants;

  /**
   * Constructs a new {@code SystemOfThreeEquations} instance.
   *
   * @param x an array of the coefficients of x for each equation (length 3).
   * @param y an array of the coefficients of y for each equation (length 3).
   * @param z an array of the coefficients of z for each equation (length 3).
   * @param d an array of the constant terms (right-hand side values) for each equation (length 3).
   * @throws IllegalArgumentException if any of the input arrays do not have exactly 3 elements.
   */
  public SystemOfThreeEquations(double[] x, double[] y, double[] z, double[] d) {
    if (x.length != 3 || y.length != 3 || z.length != 3 || d.length != 3) {
      throw new IllegalArgumentException("All input arrays must have exactly 3 elements.");
    }

    double[][] coefficientsArray = {
      {x[0], y[0], z[0]},
      {x[1], y[1], z[1]},
      {x[2], y[2], z[2]}
    };

    double[] constantsArray = {d[0], d[1], d[2]};

    this.coefficients = new Matrix(coefficientsArray);
    this.constants = new Matrix(constantsArray, 3);
  }

  /**
   * Solves the system of three linear equations.
   *
   * @return a {@link SolvedSystem} object containing the solution for x, y, and z.
   * @throws IllegalArgumentException if the determinant of the coefficient matrix is zero,
   *     indicating that the system has no unique solution.
   */
  public SolvedSystem solve() throws IllegalArgumentException {
    double determinant = coefficients.det();

    if (determinant == 0) {
      throw new IllegalArgumentException("The system of equations has no unique solution.");
    }

    return new SolvedSystem(coefficients.solve(constants));
  }
}
