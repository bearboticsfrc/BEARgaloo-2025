package bearlib.motor.deserializer.models.encoder;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.Optional;

/** Represents the configuration of an encoder. */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class Encoder {
  /** Indicates whether the encoder is inverted. */
  private Boolean inverted;

  /** The initial position of the encoder. */
  @JsonProperty("initial_position")
  private Double initialPosition;

  /** The conversion factors for position and velocity measurements. */
  @JsonProperty("conversion_factor")
  private ConversionFactor conversionFactor;

  /**
   * Gets whether the encoder is inverted.
   *
   * @return an {@link Optional} containing the inversion state, or empty if not set
   */
  public Optional<Boolean> getInverted() {
    return Optional.ofNullable(inverted);
  }

  /**
   * Sets whether the encoder is inverted.
   *
   * @param inverted the inversion state to set
   */
  public void setInverted(Boolean inverted) {
    this.inverted = inverted;
  }

  /**
   * Gets the initial position of the encoder.
   *
   * @return an {@link Optional} containing the initial position, or empty if not set
   */
  public Optional<Double> getInitialPosition() {
    return Optional.ofNullable(initialPosition);
  }

  /**
   * Sets the initial position of the encoder.
   *
   * @param initialPosition the initial position to set
   */
  public void setInitialPosition(Double initialPosition) {
    this.initialPosition = initialPosition;
  }

  /**
   * Gets the conversion factors for the encoder.
   *
   * @return an {@link Optional} containing the conversion factors, or empty if not set
   */
  public Optional<ConversionFactor> getConversionFactor() {
    return Optional.ofNullable(conversionFactor);
  }

  /**
   * Sets the conversion factors for the encoder.
   *
   * @param conversionFactor the conversion factors to set
   */
  public void setConversionFactor(ConversionFactor conversionFactor) {
    this.conversionFactor = conversionFactor;
  }
}
