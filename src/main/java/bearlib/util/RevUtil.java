// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package bearlib.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

/** Convenience methods for using the RevRobotics api. */
public class RevUtil {

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   */
  public static void checkRevError(REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(error.toString(), true);
    }
  }

  /**
   * Accepts a command that returns a REVLibError and if it is not "ok" then print an error to the
   * driver station
   *
   * @param error A REVLibError from any REV API command
   * @param message A message to log with the error.
   */
  public static void checkRevError(REVLibError error, String message) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }
  }
}
