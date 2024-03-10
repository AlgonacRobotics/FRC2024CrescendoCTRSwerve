// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeDrive;

  private SparkPIDController intake_pidController;
  private RelativeEncoder intake_Encoder;
  // private SparkAbsoluteEncoder intake_AbsoluteEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private boolean m_positionMode;
  private double m_targetPosition;
  private double m_power;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeDrive = new CANSparkMax(Constants.intakeDrive_ID, MotorType.kBrushless);

    intakeDrive.restoreFactoryDefaults();
    intakeDrive.setInverted(false);
    intakeDrive.setSmartCurrentLimit(Constants.Intake.kCurrentLimit);
    intakeDrive.setIdleMode(IdleMode.kBrake);

    // intake_pidController = intakeDrive.getPIDController();
    intake_Encoder = intakeDrive.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    intake_pidController = intakeDrive.getPIDController();

    // PID coefficients
    kP = 1;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // set PID coefficients
    intake_pidController.setP(kP);
    intake_pidController.setI(kI);
    intake_pidController.setD(kD);
    intake_pidController.setIZone(kIz);
    intake_pidController.setFF(kFF);
    intake_pidController.setOutputRange(kMinOutput, kMaxOutput);

    intakeDrive.burnFlash();

    m_positionMode = false;
    m_targetPosition = intake_Encoder.getPosition();
    m_power = 0;
  }

  public void setPower(double _power) {
    m_positionMode = false;
    m_targetPosition = intake_Encoder.getPosition();
    m_power = _power;
  }

  /**
   * Constructs a command that drives the rollers a specific distance (number of rotations) from the
   * current position and then ends the command.
   *
   * @return The retract command
   */
  public Command retract() {
    Command newCommand =
        new Command() {
          @Override
          public void initialize() {
            m_positionMode = true;
            m_targetPosition = intake_Encoder.getPosition() + Constants.Intake.kRetractDistance;
          }

          @Override
          public boolean isFinished() {
            return isNearTarget();
          }
        };

    newCommand.addRequirements(this);

    return newCommand;
  }

  public void IntakeRetractInit() {
    m_positionMode = true;
    m_targetPosition = intake_Encoder.getPosition() + Constants.Intake.kRetractDistance;
  }

  /*public void IntakeIn() {
    intakeDrive.set(.2);
  }*/

  public void IntakeOut() {
    setPower(-.2);
  }

  public void IntakeBlast() {
    intakeDrive.set(.6);
  }

 /*  public void IntakeStop() {
    intakeDrive.set(0);
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if we've reached the position target, drop out of position mode
    if (m_positionMode && isNearTarget()) {
      m_positionMode = false;
      m_power = 0.0;
    }

    // update the motor power based on mode and setpoint
    if (m_positionMode) {
      intake_pidController.setReference(m_targetPosition, ControlType.kPosition);
    } else {
      intakeDrive.set(m_power);
    }
  }

  /**
   * Check if the encoder is within the position tolerance.
   *
   * @return Whether the position is within the tolerance.
   */
  public boolean isNearTarget() {
    return Math.abs(intake_Encoder.getPosition() - m_targetPosition)
        < Constants.Intake.kPositionTolerance;
  }
}
