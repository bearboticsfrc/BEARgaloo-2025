package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorConfig {
  private SparkMax motor;
  private Object motorEncoder;
  private String moduleName;
  private String name;
  private boolean motorInverted;
  private boolean encoderInverted;
  private double nominalVoltage;
  private int currentLimit;
  private double positionConversionFactor;
  private double velocityConversionFactor;
  private SparkMaxConfig config = new SparkMaxConfig();

  /**
   * @param motor Specific motor to configure
   * @param motorEncoder Motor encoder for this motor
   * @param inverted Whether this motor should be inverted or not
   * @param nominalVoltage The nominal voltage compansation for this motor
   * @param currentLimit The current limit for this motor
   */
  public MotorConfig(
      SparkMax motor,
      Object motorEncoder,
      String moduleName,
      String name,
      boolean motorInverted,
      boolean encoderInverted,
      double nominalVoltage,
      int currentLimit,
      double positionConversionFactor,
      double velocityConversionFactor) {
    this.motor = motor;
    this.motorEncoder = motorEncoder;
    this.moduleName = moduleName;
    this.name = name;
    this.motorInverted = motorInverted;
    this.encoderInverted = encoderInverted;
    this.nominalVoltage = nominalVoltage;
    this.currentLimit = currentLimit;
    this.positionConversionFactor = positionConversionFactor;
    this.velocityConversionFactor = velocityConversionFactor;
  }

  /*
   * Generic configuration method
   */
  public MotorConfig configureMotor() {

    config.idleMode(IdleMode.kBrake).inverted(motorInverted).voltageCompensation(nominalVoltage).smartCurrentLimit(currentLimit);

    //motor.setInverted(motorInverted);
    //RevUtil.checkRevError(motor.setIdleMode(SparkBaseConfig.IdleMode.kBrake));
    //RevUtil.checkRevError(motor.enableVoltageCompensation(nominalVoltage));
    //RevUtil.checkRevError(motor.setSmartCurrentLimit(currentLimit));
    //RevUtil.setPeriodicFramePeriodHigh(motor, String.format("%s - %s", moduleName, name));

    if (motorEncoder instanceof RelativeEncoder) {
      RelativeEncoder encoder = (RelativeEncoder) motorEncoder;
      config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      config.encoder.positionConversionFactor(positionConversionFactor);
      config.encoder.velocityConversionFactor(velocityConversionFactor);
    
      ((RelativeEncoder) motorEncoder).setPosition(0);
    } else if (motorEncoder instanceof AbsoluteEncoder) {
      // Since we use AbsoluteEncoder with our
      // pivot motor it's safe to assume here.
      // Won't be the case for 100% of builds although.
      AbsoluteEncoder encoder = (AbsoluteEncoder) motorEncoder; // silly cast for silly java

      config.encoder.inverted(encoderInverted);
      config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
      config.encoder.positionConversionFactor(positionConversionFactor);
      config.encoder.velocityConversionFactor(velocityConversionFactor);
    }
    return this;
  }

  public MotorConfig configurePID(MotorPIDBuilder motorPid) {
    SparkMaxConfig config = new SparkMaxConfig();

    SparkClosedLoopController motorPIDController = motor.getClosedLoopController();
    config.closedLoop.p(motorPid.getP()).i(motorPid.getI()).d(motorPid.getD()).velocityFF(motorPid.getFf())
    .outputRange(motorPid.getMinOutput(), motorPid.getMaxOutput());

    boolean positionPidWrappingEnabled = motorPid.isPositionPidWrappingEnabled();

    if (positionPidWrappingEnabled) {
      config.closedLoop.positionWrappingEnabled(positionPidWrappingEnabled)
          .positionWrappingInputRange(motorPid.getPositionPidWrappingMin(), motorPid.getPositionPidWrappingMax());
    }

    return this;
  }

  
  public MotorConfig configurePID(MotorPIDBuilder motorPid, int slot) {
 
    ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
    if ( slot == 1 ) closedLoopSlot = ClosedLoopSlot.kSlot1;

    SparkMaxConfig config = new SparkMaxConfig();

    SparkClosedLoopController motorPIDController = motor.getClosedLoopController();

    config.closedLoop.p(motorPid.getP(),closedLoopSlot)
    .i(motorPid.getI(),closedLoopSlot)
    .d(motorPid.getD(),closedLoopSlot)
    .velocityFF(motorPid.getFf(),closedLoopSlot)
    .outputRange(motorPid.getMinOutput(), motorPid.getMaxOutput(),closedLoopSlot);

    boolean positionPidWrappingEnabled = motorPid.isPositionPidWrappingEnabled();

    if (positionPidWrappingEnabled) {
      config.closedLoop.positionWrappingEnabled(positionPidWrappingEnabled)
          .positionWrappingInputRange(motorPid.getPositionPidWrappingMin(), motorPid.getPositionPidWrappingMax());
    }

    return this;
  }

  public void burnFlash() {
    motor.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //    RevUtil.checkRevError(motor.burnFlash());

    // Burn settings onto motor flash
    // might not work, needs a delay after setting values
  }

  public static MotorConfig fromMotorConstants(
      SparkMax motor, Object motorEncoder, MotorBuilder constants) {
    return new MotorConfig(
        motor,
        motorEncoder,
        constants.getModuleName(),
        constants.getName(),
        constants.isMotorInverted(),
        constants.isEncoderInverted(),
        constants.getNominalVoltage(),
        constants.getCurrentLimit(),
        constants.getPositionConversionFactor(),
        constants.getVelocityConversionFactor());
  }

  public static class MotorPIDBuilder {
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double ff = 0;
    private double minOutput = -1;
    private double maxOutput = 1;
    private boolean positionPidWrappingEnabled = false;
    private double positionPidWrappingMin = -1;
    private double positionPidWrappingMax = 1;

    public double getP() {
      return p;
    }

    public MotorPIDBuilder setP(double p) {
      this.p = p;
      return this;
    }

    public double getI() {
      return i;
    }

    public MotorPIDBuilder setI(double i) {
      this.i = i;
      return this;
    }

    public double getD() {
      return d;
    }

    public MotorPIDBuilder setD(double d) {
      this.d = d;
      return this;
    }

    public double getFf() {
      return ff;
    }

    public MotorPIDBuilder setFf(double ff) {
      this.ff = ff;
      return this;
    }

    public double getMinOutput() {
      return minOutput;
    }

    public MotorPIDBuilder setMinOutput(double minOutput) {
      this.minOutput = minOutput;
      return this;
    }

    public double getMaxOutput() {
      return maxOutput;
    }

    public MotorPIDBuilder setMaxOutput(double maxOutput) {
      this.maxOutput = maxOutput;
      return this;
    }

    public boolean isPositionPidWrappingEnabled() {
      return positionPidWrappingEnabled;
    }

    public MotorPIDBuilder setPositionPidWrappingEnabled(boolean enabled) {
      this.positionPidWrappingEnabled = enabled;
      return this;
    }

    public double getPositionPidWrappingMin() {
      return positionPidWrappingMin;
    }

    public MotorPIDBuilder setPositionPidWrappingMin(double positionPidWrappingMin) {
      this.positionPidWrappingMin = positionPidWrappingMin;
      return this;
    }

    public double getPositionPidWrappingMax() {
      return positionPidWrappingMax;
    }

    public MotorPIDBuilder setPositionPidWrappingMax(double positionPidWrappingMax) {
      this.positionPidWrappingMax = positionPidWrappingMax;
      return this;
    }
  }

  public static class MotorBuilder {
    private String name;
    private String moduleName;
    private int motorPort;
    private boolean motorInverted;
    private boolean encoderInverted;
    private MotorPIDBuilder motorPID;
    private double nominalVoltage = 12;
    private int currentLimit;
    private double positionConversionFactor = 1;
    private double velocityConversionFactor = 1;
    private MotorPIDBuilder[] pidSlots = new MotorPIDBuilder[2];

    public MotorBuilder setMotorPid(MotorPIDBuilder motorPid, int slot) {
      pidSlots[slot] = motorPid;
      return this;
    }

    public MotorPIDBuilder getMotorPid(int slot) {
      return pidSlots[slot];
    }

    /**
     * @return the positionConversionFactor
     */
    public double getPositionConversionFactor() {
      return positionConversionFactor;
    }

    /**
     * @param positionConversionFactor the positionConversionFactor to set
     */
    public MotorBuilder setPositionConversionFactor(double positionConversionFactor) {
      this.positionConversionFactor = positionConversionFactor;
      return this;
    }

    /**
     * @return the velocityConversionFactor
     */
    public double getVelocityConversionFactor() {
      return velocityConversionFactor;
    }

    /**
     * @param velocityConversionFactor the velocityConversionFactor to set
     */
    public MotorBuilder setVelocityConversionFactor(double velocityConversionFactor) {
      this.velocityConversionFactor = velocityConversionFactor;
      return this;
    }

    public String getName() {
      return name;
    }

    public MotorBuilder setName(String name) {
      this.name = name;
      return this;
    }

    public int getMotorPort() {
      return motorPort;
    }

    public MotorBuilder setMotorPort(int motorPort) {
      this.motorPort = motorPort;
      return this;
    }

    public MotorPIDBuilder getMotorPID() {
      return motorPID;
    }

    public MotorBuilder setMotorPID(MotorPIDBuilder motorPID) {
      this.motorPID = motorPID;
      return this;
    }

    public String getModuleName() {
      return moduleName;
    }

    public MotorBuilder setModuleName(String moduleName) {
      this.moduleName = moduleName;
      return this;
    }

    public double getNominalVoltage() {
      return nominalVoltage;
    }

    public MotorBuilder setNominalVoltage(double nominalVoltage) {
      this.nominalVoltage = nominalVoltage;
      return this;
    }

    public int getCurrentLimit() {
      return currentLimit;
    }

    public MotorBuilder setCurrentLimit(int currentLimit) {
      this.currentLimit = currentLimit;
      return this;
    }

    /**
     * @return the motorInverted
     */
    public boolean isMotorInverted() {
      return motorInverted;
    }

    /**
     * @param motorInverted the motorInverted to set
     */
    public MotorBuilder setMotorInverted(boolean motorInverted) {
      this.motorInverted = motorInverted;
      return this;
    }

    /**
     * @return the encoderInverted
     */
    public boolean isEncoderInverted() {
      return encoderInverted;
    }

    /**
     * @param encoderInverted the encoderInverted to set
     */
    public MotorBuilder setEncoderInverted(boolean encoderInverted) {
      this.encoderInverted = encoderInverted;
      return this;
    }
  }
}
