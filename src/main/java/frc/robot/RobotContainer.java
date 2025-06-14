// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WristCalibrateCommand;
import frc.robot.constants.AutoConstants.ScorePosition;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.SpeedMode;
import frc.robot.constants.manipulator.RollerConstants.RollerSpeed;
import frc.robot.constants.manipulator.WristConstants.WristPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import java.util.ArrayList;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private List<Pair<String, Command>> autoList = new ArrayList<Pair<String, Command>>();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();

  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(DriveConstants.OPERATOR_CONTROLLER_PORT);

  private SendableChooser<Command> chooser = new SendableChooser<>();

  private boolean isTeleop = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem.setDefaultCommand(getDefaultCommand());
    // setManipulatorDefaultCommand();
    configureControllerMappings();
    buildAutoList();
    setupShuffleboardTab();
  }

  private RunCommand getDefaultCommand() {
    return new RunCommand(
        () ->
            driveSubsystem.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                -MathUtil.applyDeadband(driverController.getRightX(), 0.1),
                false),
        driveSubsystem);
  }

  public void setTeleop(boolean mode) {
    isTeleop = mode;
  }

  public void configureControllerMappings() {
    configureDriverController();
  }

  private void configureDriverController() {
    driverController.a().onTrue(new InstantCommand(driveSubsystem::zeroHeading));

    driverController.b().onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));

    // driverController
    //   .x()
    //    .whileTrue(manipulatorSubsystem.getCubeHuntCommand(driveSubsystem))
    //    .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    driverController.x().onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.HIGH));

    driverController.y().onTrue(manipulatorSubsystem.getHomeAllCommand());

    driverController.leftBumper().onTrue(manipulatorSubsystem.getShootCubeCommand());

    driverController
        .leftTrigger(0.1)
        .onTrue(new InstantCommand(() -> driveSubsystem.setSpeedMode(SpeedMode.TURBO)))
        .onFalse(new InstantCommand(() -> driveSubsystem.setSpeedMode((SpeedMode.NORMAL))));

    driverController
        .rightTrigger(0.1)
        .onTrue(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.INTAKE))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    driverController
        .povDown()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));

    driverController.povUp().onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.HIGH));

    driverController
        .povLeft()
        .onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.MIDDLE));

    driverController
        .povRight()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.HIGH));

    new Trigger(() -> manipulatorSubsystem.hasCube() && isTeleop)
        .onTrue(
            new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1))
                .andThen(new WaitCommand(1))
                .andThen(
                    new InstantCommand(
                        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))));
  }

  public void configureOperatorController() {
    operatorController.a().onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));
    operatorController.y().onTrue(manipulatorSubsystem.getHomeAllCommand());
    operatorController.x().onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.HIGH));

    operatorController
        .leftTrigger(0.1)
        .onTrue(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.RELEASE))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    operatorController
        .rightTrigger(0.1)
        .onTrue(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.INTAKE))
        .onFalse(manipulatorSubsystem.getRollerRunCommand(RollerSpeed.OFF));

    operatorController
        .povDown()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.BOTTOM));

    operatorController
        .povUp()
        .onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.HIGH));

    operatorController
        .povLeft()
        .onTrue(manipulatorSubsystem.getShelfScoreCommand(ScorePosition.MIDDLE));

    operatorController
        .povRight()
        .onTrue(manipulatorSubsystem.getWristRunCommand(WristPositions.HIGH));

    operatorController.leftBumper().onTrue(manipulatorSubsystem.getShootCubeCommand());
  }

  private void setupShuffleboardTab() {
    for (Pair<String, Command> command : autoList) {
      chooser.addOption(command.getFirst(), command.getSecond());
    }

    chooser.setDefaultOption(autoList.get(0).getFirst(), autoList.get(0).getSecond());
    DriveConstants.COMPETITION_TAB.add("Auto Command", chooser).withSize(4, 1).withPosition(0, 1);
    DriveConstants.COMPETITION_TAB
        .add("Wrist Calibrate Command", new WristCalibrateCommand(manipulatorSubsystem))
        .withSize(5, 1)
        .withPosition(0, 2);
  }

  private void addToAutoList(String name, Command command) {
    autoList.add(new Pair<String, Command>(name, command));
  }

  private void buildAutoList() {
    addToAutoList("0-Nothing", new InstantCommand());
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
