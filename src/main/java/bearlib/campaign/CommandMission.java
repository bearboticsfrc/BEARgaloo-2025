package bearlib.campaign;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/**
 * {@code CommandMission} serves as an adapter, allowing WPILib {@link Command} objects to be
 * integrated within the mission framework. It wraps a {@code Command} and adapts its lifecycle
 * methods to the {@code AbstractMission} interface.
 */
public class CommandMission extends AbstractMission {

  /** The wrapped WPILib {@link Command} being adapted for the mission framework. */
  private final Command command;

  /** Supplier to evaluate whether the precondition for executing the command is met. */
  private BooleanSupplier preconditionSupplier = () -> true;

  /** Supplier to evaluate whether the success condition of the command is met. */
  private BooleanSupplier successSupplier = () -> true;

  /**
   * Constructs a new {@code CommandMission} adapter.
   *
   * @param command The {@link Command} to adapt.
   */
  public CommandMission(final Command command) {
    this.command = command;
    addRequirements(command.getRequirements());
  }

  /**
   * Initializes the adapted command.
   *
   * @see edu.wpi.first.wpilibj2.command.Command#initialize()
   */
  public void initialize() {
    command.initialize();
  }

  /**
   * Executes the adapted command.
   *
   * @see edu.wpi.first.wpilibj2.command.Command#execute()
   */
  public void execute() {
    command.execute();
  }

  /**
   * Ends the adapted command.
   *
   * @param interrupted Whether the command was interrupted during execution.
   * @see edu.wpi.first.wpilibj2.command.Command#end(boolean)
   */
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  /**
   * Checks whether the adapted command has finished execution.
   *
   * @return {@code true} if the command has finished, {@code false} otherwise.
   * @see edu.wpi.first.wpilibj2.command.Command#isFinished()
   */
  public boolean isFinished() {
    return command.isFinished();
  }

  /**
   * Schedules the adapted command for execution.
   *
   * @see edu.wpi.first.wpilibj2.command.Command#schedule()
   */
  public void schedule() {
    command.schedule();
  }

  /**
   * Cancels the adapted command.
   *
   * @see edu.wpi.first.wpilibj2.command.Command#cancel()
   */
  public void cancel() {
    command.cancel();
  }

  /**
   * Checks whether the adapted command is currently scheduled.
   *
   * @return {@code true} if the command is scheduled, {@code false} otherwise.
   * @see edu.wpi.first.wpilibj2.command.Command#isScheduled()
   */
  public boolean isScheduled() {
    return command.isScheduled();
  }

  /**
   * Retrieves the name of the adapted command.
   *
   * @return The name of the command.
   * @see edu.wpi.first.wpilibj2.command.Command#getName()
   */
  public String getName() {
    return command.getName();
  }

  /**
   * Specifies a success condition for the command. The provided callback is evaluated to determine
   * if the command has successfully completed.
   *
   * @param callback A {@link BooleanSupplier} representing the success condition.
   * @return This {@code CommandMission} instance, to allow for method chaining.
   */
  public CommandMission withSuccessCallback(BooleanSupplier callback) {
    this.successSupplier = callback;
    return this;
  }

  /**
   * Evaluates the success condition of the command.
   *
   * @return {@code true} if the success condition is met, {@code false} otherwise.
   */
  public boolean isSuccess() {
    return successSupplier.getAsBoolean();
  }

  /**
   * Specifies a precondition for executing the command. The provided callback is evaluated before
   * the command's execution to determine if the conditions are met for it to proceed.
   *
   * @param callback A {@link BooleanSupplier} representing the precondition.
   * @return This {@code CommandMission} instance, to allow for method chaining.
   */
  public CommandMission withPrecondition(BooleanSupplier callback) {
    this.preconditionSupplier = callback;
    return this;
  }

  /**
   * Evaluates the precondition for executing the command.
   *
   * @return {@code true} if the precondition is met, {@code false} otherwise.
   */
  public boolean isValid() {
    return preconditionSupplier.getAsBoolean();
  }
}
