package bearlib.motor.deserializer.models.motor;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.List;
import java.util.Optional;

/** Represents the configuration of a motor. */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class Motor {

  /** The type of motor (e.g., max, flex). */
  public SparkType type;

  /** The CAN ID of the motor. */
  public Integer id;

  /** Whether the motor is inverted. */
  private Boolean inverted;

  /** The idle mode of the motor (e.g., brake or coast). */
  @JsonProperty("idle_mode")
  private IdleModeType idleMode;

  /** The nominal operating voltage of the motor. Defaults to 12.0 volts. */
  @JsonProperty("nominal_voltage")
  private Double nominalVoltage = 12.0;

  /** The current limit for the motor in amperes. */
  @JsonProperty("current_limit")
  private Integer currentLimit;

  /** A list of soft limits for the motor's movement. */
  @JsonProperty("soft_limits")
  private List<SoftLimit> softLimits;

  /** A list of hard limits for the motor's movement. */
  @JsonProperty("hard_limits")
  private List<HardLimit> hardLimits;

  /** The leader configuration for the motor, if applicable. */
  private MotorLeader leader;

  /**
   * Gets whether the motor is inverted.
   *
   * @return an {@link Optional} containing the inversion state, or empty if not set
   */
  public Optional<Boolean> getInverted() {
    return Optional.ofNullable(inverted);
  }

  /**
   * Sets whether the motor is inverted.
   *
   * @param inverted the inversion state to set
   */
  public void setInverted(Boolean inverted) {
    this.inverted = inverted;
  }

  /**
   * Gets the idle mode of the motor.
   *
   * @return an {@link Optional} containing the idle mode, or empty if not set
   */
  public Optional<IdleModeType> getIdleMode() {
    return Optional.ofNullable(idleMode);
  }

  /**
   * Sets the idle mode of the motor.
   *
   * @param idleMode the idle mode to set
   */
  public void setIdleMode(IdleModeType idleMode) {
    this.idleMode = idleMode;
  }

  /**
   * Gets the nominal voltage of the motor.
   *
   * @return an {@link Optional} containing the nominal voltage, or empty if not set
   */
  public Optional<Double> getNominalVoltage() {
    return Optional.ofNullable(nominalVoltage);
  }

  /**
   * Sets the nominal voltage of the motor.
   *
   * @param nominalVoltage the nominal voltage to set
   */
  public void setNominalVoltage(Double nominalVoltage) {
    this.nominalVoltage = nominalVoltage;
  }

  /**
   * Gets the current limit of the motor.
   *
   * @return an {@link Optional} containing the current limit, or empty if not set
   */
  public Optional<Integer> getCurrentLimit() {
    return Optional.ofNullable(currentLimit);
  }

  /**
   * Sets the current limit of the motor.
   *
   * @param currentLimit the current limit to set
   */
  public void setCurrentLimit(Integer currentLimit) {
    this.currentLimit = currentLimit;
  }

  /**
   * Gets the soft limits for the motor.
   *
   * @return an {@link Optional} containing a list of soft limits, or empty if not set
   */
  public Optional<List<SoftLimit>> getSoftLimits() {
    return Optional.ofNullable(softLimits);
  }

  /**
   * Sets the soft limits for the motor. Ensures the list contains fewer than 3 elements.
   *
   * @param softLimits the list of soft limits to set
   * @throws IllegalArgumentException if the list contains more than 2 elements
   */
  public void setSoftLimits(List<SoftLimit> softLimits) {
    if (softLimits.size() >= 3) {
      throw new IllegalArgumentException("Soft limits list must contain fewer than 3 elements.");
    }
    this.softLimits = softLimits;
  }

  /**
   * Gets the hard limits for the motor.
   *
   * @return an {@link Optional} containing a list of hard limits, or empty if not set
   */
  public Optional<List<HardLimit>> getHardLimits() {
    return Optional.ofNullable(hardLimits);
  }

  /**
   * Sets the hard limits for the motor. Ensures the list contains fewer than 3 elements.
   *
   * @param hardLimits the list of hard limits to set
   * @throws IllegalArgumentException if the list contains more than 2 elements
   */
  public void setHardLimits(List<HardLimit> hardLimits) {
    if (hardLimits.size() >= 3) {
      throw new IllegalArgumentException("Hard limits list must contain fewer than 3 elements.");
    }
    this.hardLimits = hardLimits;
  }

  /**
   * Gets the leader configuration for the motor.
   *
   * @return an {@link Optional} containing the leader configuration, or empty if not set
   */
  public Optional<MotorLeader> getLeader() {
    return Optional.ofNullable(leader);
  }

  /**
   * Sets the leader configuration for the motor.
   *
   * @param leader the leader configuration to set
   */
  public void setLeader(MotorLeader leader) {
    this.leader = leader;
  }
}
