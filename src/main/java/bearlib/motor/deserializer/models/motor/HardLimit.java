package bearlib.motor.deserializer.models.motor;

import com.fasterxml.jackson.annotation.JsonInclude;

/**
 * Represents a hard limit configuration for a motor, defining a directional hard limit (e.g., limit
 * switch) to restrict motor movement.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class HardLimit {
  /** The direction of the hard limit (e.g., forward or reverse). */
  public LimitDirection direction;

  /** The normal state of the limit switch. (e.g., open or closed) */
  public HardLimitType type;
}
