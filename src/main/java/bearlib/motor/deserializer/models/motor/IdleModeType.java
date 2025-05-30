package bearlib.motor.deserializer.models.motor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Map;

/**
 * Represents the types of idle modes for a motor configuration. These modes determine how the motor
 * behaves when no input is applied.
 */
public enum IdleModeType {

  /** Brake mode: the motor resists movement when idle. */
  BRAKE,

  /** Coast mode: the motor allows free movement when idle. */
  COAST;

  /** A mapping between IdleModeType and the corresponding IdleMode. */
  private static final Map<IdleModeType, IdleMode> idleModeMap =
      Map.of(
          BRAKE, IdleMode.kBrake,
          COAST, IdleMode.kCoast);

  /**
   * Converts this IdleModeType instance to the corresponding {@link IdleMode}.
   *
   * @return the corresponding {@link IdleMode}
   */
  public IdleMode toIdleMode() {
    return idleModeMap.get(this);
  }
}
