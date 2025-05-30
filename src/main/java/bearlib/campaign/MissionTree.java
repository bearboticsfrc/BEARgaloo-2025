package bearlib.campaign;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents a binary tree structure for organizing missions within a campaign. Each node in the
 * tree is a mission, and each mission can lead to either a success node or a failure node,
 * representing the next mission to execute based on the outcome of the current mission.
 */
public class MissionTree {
  private AbstractMission node;
  private MissionTree successNode = null;
  private MissionTree failureNode = null;

  /**
   * Constructs a MissionTree node with a specific mission.
   *
   * @param node The mission associated with this tree node.
   */
  public MissionTree(AbstractMission node) {
    this.node = node;
  }

  /**
   * Constructs a MissionTree node wrapping a Command as a mission.
   *
   * @param node The Command to wrap as a mission.
   */
  public MissionTree(Command node) {
    this.node = new CommandMission(node);
  }

  /**
   * Returns the node representing the mission to execute upon success of the current mission.
   *
   * @return The success node.
   */
  public MissionTree getSuccessNode() {
    return successNode;
  }

  /**
   * Sets the success node of this tree node and returns itself for method chaining.
   *
   * @param successNode The success node to set.
   * @return This MissionTree instance to allow for method chaining.
   */
  public MissionTree withSuccessNode(MissionTree successNode) {
    this.successNode = successNode;
    return this;
  }

  /**
   * Sets the success node of this tree node with a CommandMission and returns itself for method
   * chaining.
   *
   * @param successNode The CommandMission to wrap and set as the success node.
   * @return This MissionTree instance to allow for method chaining.
   */
  public MissionTree withSuccessNode(CommandMission successNode) {
    this.successNode = new MissionTree(successNode);
    return this;
  }

  /**
   * Returns the node representing the mission to execute upon failure of the current mission.
   *
   * @return The failure node.
   */
  public MissionTree getFailureNode() {
    return failureNode;
  }

  /**
   * Sets the failure node of this tree node and returns itself for method chaining.
   *
   * @param failureNode The failure node to set.
   * @return This MissionTree instance to allow for method chaining.
   */
  public MissionTree withFailureNode(MissionTree failureNode) {
    this.failureNode = failureNode;
    return this;
  }

  /**
   * Sets the failure node of this tree node with a CommandMission and returns itself for method
   * chaining.
   *
   * @param failureNode The CommandMission to wrap and set as the failure node.
   * @return This MissionTree instance to allow for method chaining.
   */
  public MissionTree withFailureNode(CommandMission failureNode) {
    this.failureNode = new MissionTree(failureNode);
    return this;
  }

  /**
   * Returns the current mission associated with this tree node.
   *
   * @return The current mission.
   */
  public AbstractMission getNode() {
    return node;
  }

  /**
   * Sets the current mission for this tree node.
   *
   * @param node The mission to set.
   */
  public void setNode(AbstractMission node) {
    this.node = node;
  }
}
