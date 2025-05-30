package bearlib.motor.deserializer.models.motor;

import com.revrobotics.spark.config.LimitSwitchConfig;

/** Represents the type of a hard limit. */
public enum HardLimitType {

  /** Specifies the hard limit is normally open. */
  OPEN(LimitSwitchConfig.Type.kNormallyOpen),

  /** Specifies the hard limit is normally closed. */
  CLOSED(LimitSwitchConfig.Type.kNormallyClosed);

  /** The underlying {@link LimitSwitchConfig.Type} value. */
  public final LimitSwitchConfig.Type value;

  private HardLimitType(LimitSwitchConfig.Type type) {
    this.value = type;
  }
}
