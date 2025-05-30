package bearlib.campaign;

/**
 * Represents a campaign comprising a set of missions for a robot to complete. A campaign is
 * identified by a name and contains a structured collection of missions, organized within a
 * MissionTree, defining the sequence and dependencies of missions to be executed.
 */
public class Campaign {
  private MissionTree missions;
  private String name;

  /**
   * Constructs a new Campaign instance with a specified name and a structured set of missions.
   *
   * @param name the name of the campaign, providing a unique identifier.
   * @param missions the MissionTree that holds the collection or hierarchy of missions for this
   *     campaign.
   */
  public Campaign(String name, MissionTree missions) {
    this.name = name;
    this.missions = missions;
  }

  /**
   * Retrieves the name of the campaign.
   *
   * @return the name of the campaign.
   */
  public String getName() {
    return name;
  }

  /**
   * Retrieves the mission tree associated with this campaign.
   *
   * @return the MissionTree for this campaign.
   */
  public MissionTree getMissions() {
    return missions;
  }
}
