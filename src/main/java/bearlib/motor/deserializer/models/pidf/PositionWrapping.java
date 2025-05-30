package bearlib.motor.deserializer.models.pidf;

import com.fasterxml.jackson.annotation.JsonAlias;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Represents the configuration for position wrapping, including minimum and maximum output limits.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class PositionWrapping {

  /** The minimum output limit for position wrapping. */
  @JsonProperty("minimum_output")
  @JsonAlias({"min_output", "minOutput"})
  public double minimumOutput;

  /** The maximum output limit for position wrapping. */
  @JsonProperty("maximum_output")
  @JsonAlias({"max_output", "maxOutput"})
  public double maximumOutput;
}
