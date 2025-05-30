package bearlib.campaign;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Manages the execution of a campaign, sequentially executing its missions based on their defined
 * success or failure paths. This class extends the Command framework of WPILib, allowing it to be
 * integrated into robot command scheduling. It handles mission transitions and execution based on
 * the current state of the campaign's mission tree.
 */
public class CampaignExecutor extends Command {
  private Campaign campaign;

  private MissionTree nextMissionTree = null;
  private AbstractMission currentMission = null;

  /**
   * Constructs a new CampaignExecutor with the specified campaign.
   *
   * @param campaign The campaign that this executor will manage and execute.
   */
  public CampaignExecutor(Campaign campaign) {
    this.campaign = campaign;
  }

  /**
   * Initializes the executor by setting up the first mission from the campaign's mission tree. This
   * method prepares the first mission for execution.
   */
  @Override
  public void initialize() {
    nextMissionTree = campaign.getMissions();
    currentMission = nextMissionTree.getNode();
    currentMission.initialize();
  }

  /**
   * Executes the current mission in the campaign. If the current mission is invalid or has
   * finished, it transitions to the next appropriate mission based on the outcome of the current
   * mission.
   */
  @Override
  public void execute() {
    // TODO: Smelly. Needs work.

    if (!currentMission.isValid() && nextMissionTree != null) {
      nextMissionTree = getNextMissionTree(false);

      currentMission = nextMissionTree.getNode();
      currentMission.initialize();

      return;
    }

    currentMission.execute();

    if (!currentMission.isFinished() || nextMissionTree == null) {
      return;
    }

    currentMission.end(false);
    nextMissionTree = getNextMissionTree(currentMission.isSuccess());

    if (nextMissionTree == null) {
      currentMission = null; // maybe use this.end()?
      return;
    }

    currentMission = nextMissionTree.getNode();
    currentMission.initialize();
  }

  /**
   * Retrieves the next mission tree node based on the success or failure of the previous mission.
   * This method determines the path of execution within the campaign's mission tree.
   *
   * @param previousSuccess Indicates whether the previous mission was successful.
   * @return The next mission tree node to execute, based on the outcome of the previous mission.
   */
  private MissionTree getNextMissionTree(boolean previousSuccess) {
    return previousSuccess ? nextMissionTree.getSuccessNode() : nextMissionTree.getFailureNode();
  }

  /**
   * Ends the current mission if the command execution is interrupted.
   *
   * @param interrupted Indicates if the command execution was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    if (currentMission != null) {
      currentMission.end(false);
    }
  }

  /**
   * Determines if the campaign execution is finished.
   *
   * @return true if there are no more missions to execute, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return currentMission == null;
  }
}
