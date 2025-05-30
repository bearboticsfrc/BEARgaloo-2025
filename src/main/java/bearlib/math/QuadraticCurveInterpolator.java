package bearlib.math;

import edu.wpi.first.math.MathUtil;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A utility class for performing quadratic curve interpolation based on a set of input-output
 * pairs.
 */
public class QuadraticCurveInterpolator {
  private final double C = 1; // the coefficient for c in any quadratic is one

  private final Map<Double, Double> inputMap;
  private final List<Double> inputKeys;

  /**
   * Constructs an QuadraticCurveInterpolator object with the given input-output map.
   *
   * @param inputMap The map representing the input-output pairs.
   */
  public QuadraticCurveInterpolator(Map<Double, Double> inputMap) {
    this.inputMap = inputMap;
    this.inputKeys = new ArrayList<>(inputMap.keySet());
  }

  /**
   * Calculates the output value for a given input using 3 point quadratic curve interpolation.
   *
   * @param input The input value.
   * @return The interpolated output value.
   */
  public double calculate(double input) {
    // Get the value in our map that is closest to our input
    double center = getNearestTarget(input);

    // Find the index of our value element
    int centerIndex = inputKeys.indexOf(center);

    // Find the keys to the left (idx - 1) and right (idx + 1) of our center key
    double x1 = inputKeys.get(clampIndex(centerIndex - 1));
    double x2 = center;
    double x3 = inputKeys.get(clampIndex(centerIndex + 1));

    // Find the respective values
    double y1 = inputMap.get(x1);
    double y2 = inputMap.get(x2);
    double y3 = inputMap.get(x3);

    // Build the coefficients
    double x[] = {Math.pow(x1, 2), Math.pow(x2, 2), Math.pow(x3, 2)};
    double y[] = {x1, x2, x3};
    double z[] = {C, C, C};
    double d[] = {y1, y2, y3};

    // Solve the system
    SystemOfThreeEquations system = new SystemOfThreeEquations(x, y, z, d);
    SolvedSystem solvedSystem = system.solve();

    // Calculate a quadratic based off of the calculated x, y, and z
    return new Quadratic(solvedSystem.getX(), solvedSystem.getY(), solvedSystem.getZ())
        .solve(input);
  }

  /**
   * Finds the nearest target input value to the given target value.
   *
   * @param target The target input value.
   * @return The nearest input value from the input map.
   */
  private double getNearestTarget(double target) {
    return inputKeys.stream()
        .min(Comparator.comparingDouble(key -> Math.abs(target - key)))
        .orElse(0.0);
  }

  /**
   * Clamp the provided index to always be in-bounds.
   *
   * @param index The index to clamp.
   * @return The clamped index.
   */
  private int clampIndex(int index) {
    return MathUtil.clamp(index, 0, inputKeys.size() - 1);
  }
}
