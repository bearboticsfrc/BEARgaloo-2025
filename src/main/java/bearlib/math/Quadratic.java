package bearlib.math;

/** Represents a quadratic function of the form: ax^2 + bx + c */
public class Quadratic {
  private double a;
  private double b;
  private double c;

  /**
   * Constructs a quadratic function with the given coefficients.
   *
   * @param a The coefficient 'a' in the quadratic equation.
   * @param b The coefficient 'b' in the quadratic equation.
   * @param c The constant term 'c' in the quadratic equation.
   */
  public Quadratic(double a, double b, double c) {
    this.a = a;
    this.b = b;
    this.c = c;
  }

  /**
   * Calculates the output value of the quadratic function for the given input.
   *
   * @param input The input value for which to evaluate the quadratic function.
   * @return The output value of the quadratic function.
   */
  public double solve(double input) {
    return (a * Math.pow(input, 2)) + (b * input) + c;
  }
}
