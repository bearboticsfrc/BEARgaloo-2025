package bearlib.motor.deserializer.models.pidf;

import com.fasterxml.jackson.annotation.JsonAlias;

public enum FeedbackSensor {
  @JsonAlias({"AbsoluteEncoder"})
  ABSOLUTE_ENCODER,

  @JsonAlias({"PrimaryEncoder"})
  PRIMARY_ENCODER
}
