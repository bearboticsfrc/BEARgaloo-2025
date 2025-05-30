package bearlib.motor.deserializer.models.motor;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.Optional;

/** Represents the leader configuration for a motor. */
@JsonInclude(JsonInclude.Include.NON_NULL)
public class MotorLeader {

  /** The CAN ID of the leader motor. */
  @JsonProperty("id")
  public Integer id;

  /** Indicates whether the follower motor should invert its behavior to match the leader. */
  @JsonProperty("follow_inverted")
  private Boolean followInverted;

  /**
   * Gets whether the follower motor is inverted relative to the leader.
   *
   * @return an {@link Optional} containing the inversion state, or empty if not set
   */
  public Optional<Boolean> getFollowInverted() {
    return Optional.ofNullable(followInverted);
  }

  /**
   * Sets whether the follower motor should invert its behavior relative to the leader.
   *
   * @param followInverted the inversion state to set
   */
  public void setFollowInverted(Boolean followInverted) {
    this.followInverted = followInverted;
  }
}
