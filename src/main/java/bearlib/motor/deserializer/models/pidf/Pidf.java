package bearlib.motor.deserializer.models.pidf;

import com.fasterxml.jackson.annotation.JsonAlias;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.Optional;

/**
 * Represents a PIDF configuration model, used for deserializing JSON data. This model includes
 * proportional (P), integral (I), derivative (D), feed-forward (FF) values.
 */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class Pidf {

  @JsonProperty("feedback_sensor")
  private FeedbackSensor feedbackSensor = FeedbackSensor.PRIMARY_ENCODER;

  /** Proportional gain value. */
  private Double p;

  /** Integral gain value. */
  private Double i;

  /** Integral zone, defining the range where integral is applied. */
  @JsonProperty("i_zone")
  private Double iZone;

  /** Derivative gain value. */
  private Double d;

  /** Feed-forward value. */
  private Double ff;

  /** Minimum output limit. */
  @JsonProperty("minimum_output")
  @JsonAlias({"min_output", "minOutput"})
  private Double minimumOutput;

  /** Maximum output limit. */
  @JsonProperty("maximum_output")
  @JsonAlias({"max_output", "maxOutput"})
  private Double maximumOutput;

  /** Configuration for position wrapping. */
  @JsonProperty("position_wrapping")
  private PositionWrapping positionWrapping;

  public Optional<FeedbackSensor> getFeedbackSensor() {
    return Optional.ofNullable(feedbackSensor);
  }

  public void setFeedbackSensor(FeedbackSensor feedbackSensor) {
    this.feedbackSensor = feedbackSensor;
  }

  /**
   * Gets the proportional gain value.
   *
   * @return an Optional containing the proportional gain value, or empty if not set
   */
  public Optional<Double> getP() {
    return Optional.ofNullable(p);
  }

  /**
   * Sets the proportional gain value.
   *
   * @param p the proportional gain value to set
   */
  public void setP(Double p) {
    this.p = p;
  }

  /**
   * Gets the integral gain value.
   *
   * @return an Optional containing the integral gain value, or empty if not set
   */
  public Optional<Double> getI() {
    return Optional.ofNullable(i);
  }

  /**
   * Sets the integral gain value.
   *
   * @param i the integral gain value to set
   */
  public void setI(Double i) {
    this.i = i;
  }

  /**
   * Gets the derivative gain value.
   *
   * @return an Optional containing the derivative gain value, or empty if not set
   */
  public Optional<Double> getD() {
    return Optional.ofNullable(d);
  }

  /**
   * Sets the derivative gain value.
   *
   * @param d the derivative gain value to set
   */
  public void setD(Double d) {
    this.d = d;
  }

  /**
   * Gets the feed-forward value.
   *
   * @return an Optional containing the feed-forward value, or empty if not set
   */
  public Optional<Double> getFf() {
    return Optional.ofNullable(ff);
  }

  /**
   * Sets the feed-forward value.
   *
   * @param ff the feed-forward value to set
   */
  public void setFf(Double ff) {
    this.ff = ff;
  }

  /**
   * Gets the minimum output limit.
   *
   * @return an Optional containing the minimum output limit, or empty if not set
   */
  public Optional<Double> getMinimumOutput() {
    return Optional.ofNullable(minimumOutput);
  }

  /**
   * Sets the minimum output limit.
   *
   * @param minimumOutput the minimum output limit to set
   */
  public void setMinimumOutput(Double minimumOutput) {
    this.minimumOutput = minimumOutput;
  }

  /**
   * Gets the maximum output limit.
   *
   * @return an Optional containing the maximum output limit, or empty if not set
   */
  public Optional<Double> getMaximumOutput() {
    return Optional.ofNullable(maximumOutput);
  }

  /**
   * Sets the maximum output limit.
   *
   * @param maximumOutput the maximum output limit to set
   */
  public void setMaximumOutput(Double maximumOutput) {
    this.maximumOutput = maximumOutput;
  }

  /**
   * Gets the position wrapping configuration.
   *
   * @return an Optional containing the position wrapping configuration, or empty if not set
   */
  public Optional<PositionWrapping> getPositionWrapping() {
    return Optional.ofNullable(positionWrapping);
  }

  /**
   * Sets the position wrapping configuration.
   *
   * @param positionWrapping the position wrapping configuration to set
   */
  public void setPositionWrapping(PositionWrapping positionWrapping) {
    this.positionWrapping = positionWrapping;
  }

  /**
   * Gets the integral zone value.
   *
   * @return an Optional containing the integral zone value, or empty if not set
   */
  public Optional<Double> getIZone() {
    return Optional.ofNullable(iZone);
  }

  /**
   * Sets the integral zone value.
   *
   * @param iZone the integral zone value to set
   */
  public void setiZone(Double iZone) {
    this.iZone = iZone;
  }
}
