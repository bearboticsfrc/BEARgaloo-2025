package bearlib.motor.deserializer.models.encoder;

import com.fasterxml.jackson.annotation.JsonInclude;
import java.util.Optional;

/** Represents the conversion factors for an encoder. */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class ConversionFactor {

  /** The conversion factor for position measurements. */
  private Double position;

  /** The conversion factor for velocity measurements. */
  private Double velocity;

  /**
   * Retrieves the conversion factor for position measurements.
   *
   * @return an {@link Optional} containing the position conversion factor, or empty if not set
   */
  public Optional<Double> getPosition() {
    return Optional.ofNullable(position);
  }

  /**
   * Sets the conversion factor for position measurements.
   *
   * @param position the position conversion factor to set
   */
  public void setPosition(Double position) {
    this.position = position;
  }

  /**
   * Retrieves the conversion factor for velocity measurements.
   *
   * @return an {@link Optional} containing the velocity conversion factor, or empty if not set
   */
  public Optional<Double> getVelocity() {
    return Optional.ofNullable(velocity);
  }

  /**
   * Sets the conversion factor for velocity measurements.
   *
   * @param velocity the velocity conversion factor to set
   */
  public void setVelocity(Double velocity) {
    this.velocity = velocity;
  }
}
