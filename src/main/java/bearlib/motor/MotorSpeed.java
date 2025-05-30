package bearlib.motor;

/**
 * Enum representing different speed levels for commonly used interfaces to configure the speed of a
 * speed controller.
 */
public enum MotorSpeed {

  /** Full reverse speed (-1.0). */
  REVERSE_FULL(-1),

  /** Three-quarters reverse speed (-0.75). */
  REVERSE_THREE_QUARTERS(-.75),

  /** Half reverse speed (-0.5). */
  REVERSE_HALF(-.5),

  /** Quarter reverse speed (-0.25). */
  REVERSE_QUARTER(-.25),

  /** Tenth reverse speed (-0.1). */
  REVERSE_TENTH(-.1),

  /** Motor is turned off (0.0). */
  OFF(0),

  /** Tenth forward speed (0.1). */
  TENTH(.1),

  /** Quarter forward speed (0.25). */
  QUARTER(.25),

  /** Half forward speed (0.5). */
  HALF(.5),

  /** Three-quarters forward speed (0.75). */
  THREE_QUARTERS(.75),

  /** Full forward speed (1.0). */
  FULL(1);

  private double speed;

  /**
   * Constructor to initialize the motor speed with a specific value.
   *
   * @param speed The speed value corresponding to the motor.
   */
  private MotorSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Gets the speed value of this motor speed.
   *
   * @return The speed value associated with this enum constant.
   */
  public double getSpeed() {
    return speed;
  }
}
