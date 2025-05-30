package bearlib.motor.deserializer.models.motor;

import com.fasterxml.jackson.annotation.JsonInclude;

/**
 * Represents a soft limit configuration for a motor, defining a directional limit to restrict motor
 * movement.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class SoftLimit {

  /** The direction of the soft limit (e.g., forward or reverse). */
  public LimitDirection direction;

  /** The positional limit in the specified direction. */
  public Double limit;
}
