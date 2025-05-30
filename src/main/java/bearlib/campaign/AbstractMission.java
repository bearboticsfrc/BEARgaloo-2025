package bearlib.campaign;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents an abstract base class for defining missions in a robotics campaign. A mission
 * encapsulates a specific task or sequence of tasks to be executed by the robot, which are defined
 * as commands in WPILib. Implementing classes must define the criteria for mission success and
 * validity.
 */
public abstract class AbstractMission extends Command {

  /**
   * Determines if the mission has successfully completed.
   *
   * @return true if the mission is successful, false otherwise.
   */
  public abstract boolean isSuccess();

  /**
   * Validates if the mission's precondition is valid and can be executed.
   *
   * @return true if the mission's precondition passed and can be executed, false otherwise.
   */
  public abstract boolean isValid();
}
