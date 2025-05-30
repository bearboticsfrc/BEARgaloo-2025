package bearlib.math;

import Jama.Matrix;

/** Represents a solved system of linear equations. */
public class SolvedSystem {
  private double x;
  private double y;
  private double z;

  /**
   * Constructs a new instance of SolvedSystem with the provided solution matrix.
   *
   * @param solution The matrix representing the solution of the system of equations. The first
   *     column of the matrix corresponds to variable x, the second column corresponds to variable
   *     y, and the third column corresponds to variable z.
   */
  public SolvedSystem(Matrix solution) {
    x = solution.get(0, 0);
    y = solution.get(1, 0);
    z = solution.get(2, 0);
  }

  /**
   * Gets the solution for the variable x.
   *
   * @return The value of variable x.
   */
  public double getX() {
    return x;
  }

  /**
   * Gets the solution for the variable y.
   *
   * @return The value of variable y.
   */
  public double getY() {
    return y;
  }

  /**
   * Gets the solution for the variable z.
   *
   * @return The value of variable z.
   */
  public double getZ() {
    return z;
  }
}
