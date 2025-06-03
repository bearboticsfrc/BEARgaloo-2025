package bearlib.motor;

import bearlib.motor.deserializer.models.encoder.ConversionFactor;
import bearlib.motor.deserializer.models.encoder.Encoder;
import bearlib.motor.deserializer.models.motor.HardLimit;
import bearlib.motor.deserializer.models.motor.IdleModeType;
import bearlib.motor.deserializer.models.motor.Motor;
import bearlib.motor.deserializer.models.motor.MotorLeader;
import bearlib.motor.deserializer.models.motor.SoftLimit;
import bearlib.motor.deserializer.models.motor.SparkType;
import bearlib.motor.deserializer.models.pidf.Pidf;
import bearlib.motor.deserializer.models.pidf.PositionWrapping;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.List;

/**
 * The {@code MotorConfigurator} class provides an interface for configuring motors, encoders, and
 * PIDF controllers using the REV Robotics Spark platform.*
 */
public class MotorConfigurator {
  private SparkBaseConfig config;
  private SparkBase spark;

  /**
   * Configures the motor..
   *
   * @param motor The {@link Motor} dataclass containing motor configuration details
   * @return the {@code MotorConfigurator} instance for chaining
   * @throws IllegalArgumentException if the motor type is unsupported
   */
  public MotorConfigurator withMotor(Motor motor) {
    setSparkAndConfig(motor.type, motor.id);

    motor.getInverted().ifPresent(config::inverted);
    motor.getIdleMode().map(IdleModeType::toIdleMode).ifPresent(config::idleMode);
    motor.getNominalVoltage().ifPresent(config::voltageCompensation);
    motor.getCurrentLimit().ifPresent(config::smartCurrentLimit);
    motor.getSoftLimits().ifPresent(this::configureSoftLimits);
    motor.getHardLimits().ifPresent(this::configureHardLimits);
    motor.getLeader().ifPresent(this::configureMotorLeader);

    return this;
  }

  /**
   * Configures the motor leader.
   *
   * @param leader the {@link MotorLeader} dataclass specifying the leader motor details
   */
  private void configureMotorLeader(MotorLeader leader) {
    config.follow(leader.id, leader.getFollowInverted().orElse(false));
  }

  /**
   * Configures soft limits for the motor.
   *
   * @param softLimits a list of {@link SoftLimit} dataclasses specifying forward and reverse limits
   * @throws IllegalArgumentException if an unsupported soft limit direction is encountered
   */
  private void configureSoftLimits(List<SoftLimit> softLimits) {
    config.softLimit.forwardSoftLimitEnabled(false);
    config.softLimit.reverseSoftLimitEnabled(false);

    for (SoftLimit softLimit : softLimits) {
      switch (softLimit.direction) {
        case FORWARD -> {
          config.softLimit.forwardSoftLimitEnabled(true);
          config.softLimit.forwardSoftLimit(softLimit.limit);
        }
        case REVERSE -> {
          config.softLimit.reverseSoftLimitEnabled(true);
          config.softLimit.reverseSoftLimit(softLimit.limit);
        }
        default ->
            throw new IllegalArgumentException(
                "Unsupported soft limit direction: " + softLimit.direction);
      }
    }
  }

  /**
   * Configures hard limits for the motor.
   *
   * @param hardLimits a list of {@link HardLimit} dataclasses specifying forward and reverse limits
   * @throws IllegalArgumentException if an unsupported hard limit direction is encountered
   */
  private void configureHardLimits(List<HardLimit> hardLimits) {
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.reverseLimitSwitchEnabled(false);

    for (HardLimit hardLimit : hardLimits) {
      switch (hardLimit.direction) {
        case FORWARD -> {
          config.limitSwitch.forwardLimitSwitchEnabled(true);
          config.limitSwitch.forwardLimitSwitchType(hardLimit.type.value);
        }
        case REVERSE -> {
          config.limitSwitch.reverseLimitSwitchEnabled(true);
          config.limitSwitch.reverseLimitSwitchType(hardLimit.type.value);
        }
        default ->
            throw new IllegalArgumentException(
                "Unsupported hard limit direction: " + hardLimit.direction);
      }
    }
  }

  /**
   * Initializes the {@code SparkBase} and {@code SparkBaseConfig} based on the motor type.
   *
   * @param type the {@link SparkType} of the motor
   * @param id the unique ID of the motor
   * @throws IllegalArgumentException if the motor type is unsupported
   */
  private void setSparkAndConfig(SparkType type, int id) {
    switch (type) {
      case MAX -> {
        spark = new SparkMax(id, MotorType.kBrushless);
        config = new SparkMaxConfig();
      }
      case FLEX -> {
        spark = new SparkFlex(id, MotorType.kBrushless);
        config = new SparkFlexConfig();
      }
      default -> throw new IllegalArgumentException("Unsupported SparkType: " + type);
    }
  }

  /**
   * Configures the encoder with the specified parameters.
   *
   * @param encoder the {@link Encoder} dataclass containing encoder configuration details
   * @return the {@code MotorConfigurator} instance for chaining
   * @throws IllegalArgumentException if the encoder type is unsupported
   */
  public MotorConfigurator withEncoder(Encoder encoder) {
    encoder.getInitialPosition().ifPresent(spark.getEncoder()::setPosition);
    encoder.getConversionFactor().ifPresent(this::configureConversionFactor);

    return this;
  }

  /**
   * Configures the position and velocity conversion factors for the encoder.
   *
   * @param conversionFactor the {@link ConversionFactor} dataclass containing conversion factors
   */
  private void configureConversionFactor(ConversionFactor conversionFactor) {
    conversionFactor.getPosition().ifPresent(config.encoder::positionConversionFactor);
    conversionFactor.getVelocity().ifPresent(config.encoder::velocityConversionFactor);
  }

  /**
   * Configures the PIDF controller for the motor.
   *
   * @param pidf the {@link Pidf} dataclass containing PIDF parameters
   * @param slot the slot index for the PIDF configuration
   * @return the {@code MotorConfigurator} instance for chaining
   * @throws IllegalArgumentException if the slot index is invalid
   */
  public MotorConfigurator withPidf(Pidf pidf, int slot) {
    ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.values()[slot];

    pidf.getFeedbackSensor()
        .ifPresent(
            feedbackSensor ->
                config.closedLoop.feedbackSensor(
                    feedbackSensor
                            == bearlib.motor.deserializer.models.pidf.FeedbackSensor
                                .ABSOLUTE_ENCODER
                        ? FeedbackSensor.kAbsoluteEncoder
                        : FeedbackSensor.kPrimaryEncoder));

    pidf.getP().ifPresent(p -> config.closedLoop.p(p, closedLoopSlot));
    pidf.getI().ifPresent(i -> config.closedLoop.i(i, closedLoopSlot));
    pidf.getIZone().ifPresent(iZone -> config.closedLoop.iZone(iZone, closedLoopSlot));
    pidf.getD().ifPresent(d -> config.closedLoop.d(d, closedLoopSlot));
    pidf.getFf().ifPresent(ff -> config.closedLoop.velocityFF(ff, closedLoopSlot));
    pidf.getMinimumOutput()
        .ifPresent(output -> config.closedLoop.minOutput(output, closedLoopSlot));
    pidf.getMaximumOutput()
        .ifPresent(output -> config.closedLoop.maxOutput(output, closedLoopSlot));
    pidf.getPositionWrapping().ifPresent(this::configurePositionWrapping);

    return this;
  }

  /**
   * Configures the PIDF controller in the default slot (slot 0).
   *
   * @param pidf the {@link Pidf} dataclass containing PIDF parameters
   */
  public void withPidf(Pidf pidf) {
    withPidf(pidf, 0);
  }

  /**
   * Configures position wrapping for the PIDF controller.
   *
   * @param positionWrapping the {@link PositionWrapping} dataclass specifying input range
   */
  private void configurePositionWrapping(PositionWrapping positionWrapping) {
    config.closedLoop.positionWrappingEnabled(true);
    config.closedLoop.positionWrappingInputRange(
        positionWrapping.minimumOutput, positionWrapping.maximumOutput);
  }

  /**
   * Applies the motor configuration asynchronously and returns a {@link SparkBase} instance.
   *
   * @return the configured motor
   */
  public SparkBase configureAsync() {
    // spark.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    spark.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    return spark;
  }
}
