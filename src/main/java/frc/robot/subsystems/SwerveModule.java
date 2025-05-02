package frc.robot.subsystems;

import bearlib.motor.deserializer.MotorParser;
import bearlib.motor.deserializer.models.encoder.ConversionFactor;
import bearlib.motor.deserializer.models.encoder.Encoder;
import bearlib.motor.deserializer.models.pidf.Pidf;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveModuleConstants;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

/** A SwerveModules consists of a drive motor and a steer motor */
public class SwerveModule {
  private final boolean SHUFFLEBOARD_ENABLED = false;

  private String moduleName;

  private SparkBase driveMotor;
  private SparkBase pivotMotor;

  private RelativeEncoder driveMotorEncoder;
  private AbsoluteEncoder pivotMotorEncoder;

  private SparkClosedLoopController driveMotorPIDController;
  private SparkClosedLoopController pivotMotorPIDController;

  private Rotation2d referenceAngle = new Rotation2d();
  private Rotation2d parkedAngle;
  private Rotation2d chassisAngularOffset;

  private boolean parked = false;

  private HashMap<String, DoubleLogEntry> dataLogs = new HashMap<String, DoubleLogEntry>();

  public SwerveModule(SwerveModuleBuilder swerveModule, ShuffleboardTab shuffleboardTab) {
    this.moduleName = swerveModule.getModuleName();
    this.parkedAngle = swerveModule.getParkAngle();
    this.chassisAngularOffset = swerveModule.getChassisAngularOffset();

    File directory = new File(Filesystem.getDeployDirectory(), "motors/drive");

    try {
      MotorParser parser =
          new MotorParser(directory).withMotor(swerveModule.moduleName + "drive.json");
      try {
        Field encoderField = MotorParser.class.getDeclaredField("encoder");
        encoderField.setAccessible(true);
        Encoder encoderConfig = (Encoder) encoderField.get(parser);
        ConversionFactor conversionFactor = new ConversionFactor();
        conversionFactor.setPosition(SwerveModuleConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        conversionFactor.setVelocity(SwerveModuleConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        encoderConfig.setConversionFactor(conversionFactor);
        encoderConfig.setInverted(false);
      } catch (Exception ex) {
        throw new RuntimeException("Failed to configure enocder on drive motor!", ex);
      }

      try {
        Field pidfField = MotorParser.class.getDeclaredField("pidfs");
        pidfField.setAccessible(true);
        Pidf[] pidfs = (Pidf[]) pidfField.get(parser);
        pidfs[0].setP(0.04);
        pidfs[0].setFf(1 / DriveConstants.DRIVE_WHEEL_FREE_SPEED_RPS);
      } catch (Exception ex) {
        throw new RuntimeException("Failed to configure pidf on drive motor!", ex);
      }

      driveMotor = parser.configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure drive motor!", exception);
    }

    try {
      MotorParser parser =
          new MotorParser(directory)
              .withMotor(swerveModule.moduleName + "pivot.json")
              .withPidf(swerveModule.moduleName + "pivot_pidf.json");
      try {
        Field encoderField = MotorParser.class.getDeclaredField("encoder");
        encoderField.setAccessible(true);
        Encoder encoderConfig = (Encoder) encoderField.get(parser);
        ConversionFactor conversionFactor = new ConversionFactor();
        conversionFactor.setPosition(SwerveModuleConstants.PIVOT_POSITION_CONVERSION_FACTOR);
        conversionFactor.setVelocity(SwerveModuleConstants.PIVOT_VELOCITY_CONVERSION_FACTOR);
        encoderConfig.setConversionFactor(conversionFactor);
        encoderConfig.setInverted(true);
      } catch (Exception ex) {
        throw new RuntimeException("Failed to configure enocder on pivot motor!", ex);
      }
      pivotMotor = parser.configureAsync();
    } catch (IOException exception) {
      throw new RuntimeException("Failed to configure pivot motor!", exception);
    }

    this.driveMotorEncoder = driveMotor.getEncoder();
    this.pivotMotorEncoder = pivotMotor.getAbsoluteEncoder();

    this.driveMotorPIDController = driveMotor.getClosedLoopController();
    this.pivotMotorPIDController = pivotMotor.getClosedLoopController();

    if (SHUFFLEBOARD_ENABLED) {
      setupShuffleboardTab(shuffleboardTab);
    }

    setupDataLogging(DataLogManager.getLog());
  }

  /**
   * @param shuffleboardTab The shuffleboard tab to use
   */
  private void setupShuffleboardTab(ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .addNumber(String.format("%s Vel", moduleName), this::getDriveVelocity)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Drive Out", moduleName), driveMotor::getAppliedOutput)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Pos", moduleName), this::getDistance)
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Steer Deg", moduleName), () -> getSteerAngle().getDegrees())
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s AE Deg", moduleName), () -> getAbsoluteAngle().getDegrees())
        .withSize(1, 1);
    shuffleboardTab
        .addNumber(String.format("%s Ref Deg", moduleName), () -> referenceAngle.getDegrees())
        .withSize(1, 1);
  }

  /**
   * Setup data logging
   *
   * @param log The log to use
   */
  private void setupDataLogging(DataLog log) {
    // TODO: refactor this, maybe

    for (String motorType : new String[] {"DRIVE", "PIVOT"}) {
      String pathMotorType = motorType.toLowerCase();

      if (motorType.equals("PIVOT")) {
        dataLogs.put(
            String.format("%s_MOTOR_POSITION", motorType),
            new DoubleLogEntry(
                log, String.format("/drive/%s/%s_motor/position", moduleName, pathMotorType)));
      }

      dataLogs.put(
          String.format("%s_MOTOR_CURRENT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/current", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_VELOCITY", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/velocity", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_APPLIED_OUTPUT", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/applied_output", moduleName, pathMotorType)));

      dataLogs.put(
          String.format("%s_MOTOR_TEMPERATURE", motorType),
          new DoubleLogEntry(
              log, String.format("/drive/%s/%s_motor/temperature", moduleName, pathMotorType)));
    }
  }

  /** Updates data logs */
  public void updateDataLogs() {
    for (Entry<String, DoubleLogEntry> entry : dataLogs.entrySet()) {
      final SparkBase motor = entry.getKey().startsWith("PIVOT") ? pivotMotor : driveMotor;
      final String property =
          entry
              .getKey()
              .replaceAll("^(PIVOT_MOTOR_|DRIVE_MOTOR_)", ""); // PIVOT_MOTOR_POSITION -> POSITION

      DoubleSupplier propertySupplier = getPropertySupplier(motor, property);
      entry.getValue().append(propertySupplier.getAsDouble());
    }
  }

  /**
   * Returns the respective getter for <b>property</b>
   *
   * @param property The property
   * @return The getter, wrapped as a DoubleSupplier
   */
  public DoubleSupplier getPropertySupplier(SparkBase motor, String property) {
    switch (property) {
      case "CURRENT":
        return motor::getOutputCurrent;
      case "VELOCITY":
        return motor.getEncoder()::getVelocity;
      case "APPLIED_OUTPUT":
        return motor::getAppliedOutput;
      case "TEMPERATURE":
        return motor::getMotorTemperature;
      case "POSITION":
        return pivotMotorEncoder::getPosition;
      default:
        throw new IllegalArgumentException("Unknown motor property: " + property);
    }
  }

  public Rotation2d getParkedAngle() {
    return parkedAngle;
  }

  /**
   * A Rotation2d representation of the angle of the steer absolute encoder
   *
   * @return The angle
   */
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRadians(pivotMotorEncoder.getPosition());
  }

  /**
   * Returns the current steer angle
   *
   * @return Rotation2d of the angle
   */
  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRadians(pivotMotorEncoder.getPosition());
  }

  /**
   * Returns the current drive veloticty
   *
   * @return The veloticty
   */
  public double getDriveVelocity() {
    return driveMotorEncoder.getVelocity();
  }

  /**
   * Returns a Rotation2d representation of the position of the drive encoder
   *
   * @return The position
   */
  public double getDistance() {
    return driveMotorEncoder.getPosition();
  }

  /**
   * Returns the position of the swerve module
   *
   * @return The position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotorEncoder.getPosition(),
        new Rotation2d(pivotMotorEncoder.getPosition() - chassisAngularOffset.getRadians()));
  }

  public void setParked(boolean mode) {
    parked = mode;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
  }

  /**
   * Sets the steer angle in radians
   *
   * @param state The state of the swerve module
   */
  public void set(SwerveModuleState state) {
    if (parked) {
      return;
    }

    SwerveModuleState desiredState =
        new SwerveModuleState(state.speedMetersPerSecond, state.angle.plus(chassisAngularOffset));
    desiredState = SwerveModuleState.optimize(desiredState, getSteerAngle());

    pivotMotorPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    driveMotorPIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);

    referenceAngle = desiredState.angle;
  }

  public static class SwerveModuleBuilder {
    private String moduleName;
    private Rotation2d parkAngle;
    private Rotation2d chassisAngularOffset;

    public String getModuleName() {
      return moduleName;
    }

    public SwerveModuleBuilder setModuleName(String moduleName) {
      this.moduleName = moduleName;
      return this;
    }

    public Rotation2d getParkAngle() {
      return parkAngle;
    }

    public SwerveModuleBuilder setParkAngle(Rotation2d parkAngle) {
      this.parkAngle = parkAngle;
      return this;
    }

    public Rotation2d getChassisAngularOffset() {
      return chassisAngularOffset;
    }

    public SwerveModuleBuilder setChassisAngularOffset(Rotation2d chassisAngularOffset) {
      this.chassisAngularOffset = chassisAngularOffset;
      return this;
    }
  }
}
