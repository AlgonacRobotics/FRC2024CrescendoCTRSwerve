// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElbowSubsystem extends SubsystemBase {

  private CANSparkMax elbowDrive;

  private SparkPIDController elbow_pidController;
  private RelativeEncoder elbow_Encoder;
  // private SparkAbsoluteEncoder elbow_AbsoluteEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_startState;
  private TrapezoidProfile.State m_endState;

  private TrapezoidProfile.State m_targetState;
  private double m_feedforward;
  private double m_manualValue;

  /** Creates a new ElbowSubsystem. */
  public ElbowSubsystem() {
    elbowDrive = new CANSparkMax(Constants.elbowDrive_ID, MotorType.kBrushless);

    elbowDrive.restoreFactoryDefaults();

    elbowDrive.setInverted(false);
    elbowDrive.setSmartCurrentLimit(Constants.Arm.kCurrentLimit);
    elbowDrive.setIdleMode(IdleMode.kBrake);
    elbowDrive.enableSoftLimit(SoftLimitDirection.kForward, true);
    elbowDrive.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elbowDrive.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.kSoftLimitForward);
    elbowDrive.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.kSoftLimitReverse);

    elbow_Encoder = elbowDrive.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    elbow_Encoder.setPositionConversionFactor(Constants.Arm.kPositionFactor);
    elbow_Encoder.setVelocityConversionFactor(Constants.Arm.kVelocityFactor);
    elbow_Encoder.setPosition(0.0);

    elbow_pidController = elbowDrive.getPIDController();
    // elbow_AbsoluteEncoder = elbowDrive.getAbsoluteEncoder(Type.kDutyCycle);
    // elbow_pidController.setFeedbackDevice(elbow_AbsoluteEncoder);

    // PID coefficients
    kP = 2.5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    elbow_pidController.setP(kP);
    elbow_pidController.setI(kI);
    elbow_pidController.setD(kD);
    elbow_pidController.setIZone(kIz);
    elbow_pidController.setFF(kFF);
    elbow_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // elbowDrive.setIdleMode(IdleMode.kBrake);
    elbowDrive.burnFlash();

    m_setpoint = Constants.Arm.kHomePosition;

    m_timer = new Timer();
    m_timer.start();

    updateMotionProfile();
  }

  /**
   * Update the motion profile variables based on the current setpoint and the pre-configured motion
   * constraints.
   */
  private void updateMotionProfile() {
    m_startState =
        new TrapezoidProfile.State(elbow_Encoder.getPosition(), elbow_Encoder.getVelocity());
    m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
    m_timer.reset();
  }

  /**
   * Sets the target position and updates the motion profile if the target position changed.
   *
   * @param _setpoint The new target position in radians.
   */
  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  /**
   * Drives the arm to a position using a trapezoidal motion profile. This function is usually
   * wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
   *
   * <p>This function updates the motor position control loop using a setpoint from the trapezoidal
   * motion profile. The target position is the last set position with {@code setTargetPosition}.
   */
  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    } else {
      m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
    }

    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            elbow_Encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset,
            m_targetState.velocity);
    elbow_pidController.setReference(
        m_targetState.position, CANSparkMax.ControlType.kPosition, 0, m_feedforward);
  }

  /**
   * Drives the arm using the provided power value (usually from a joystick). This also adds in the
   * feedforward value which can help counteract gravity.
   *
   * @param _power The motor power to apply.
   */
  public void runManual(double _power) {
    // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
    // passively
    m_setpoint = elbow_Encoder.getPosition();
    updateMotionProfile();
    // update the feedforward variable with the newly zero target velocity
    m_feedforward =
        Constants.Arm.kArmFeedforward.calculate(
            elbow_Encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset,
            m_targetState.velocity);
    // set the power of the motor
    elbowDrive.set(_power + (m_feedforward / 12.0));
    m_manualValue = _power; // this variable is only used for logging or debugging if needed
  }

  public void ElbowUp() {
    elbowDrive.set(.1);
  }

  public void ElbowDown() {
    elbowDrive.set(-.1);
  }

  public void ElbowStop() {
    elbowDrive.set(0);
  }

  public void ElbowScoreAuto() {
    // elbow_pidController.setReference(.5, CANSparkMax.ControlType.kDutyCycle);
    setTargetPosition(Constants.Arm.kScoringPosition);
  }

  public void ElbowClimbAuto() {
    // elbow_pidController.setReference(.5, CANSparkMax.ControlType.kDutyCycle);
    setTargetPosition(Constants.Arm.kIntakePosition);
  }

  public void ElbowCollectAuto() {
    // elbow_pidController.setReference(.5, CANSparkMax.ControlType.kDutyCycle);
    setTargetPosition(Constants.Arm.kIntakePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
