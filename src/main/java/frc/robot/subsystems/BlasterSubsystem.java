// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BlasterSubsystem extends SubsystemBase {

  private CANSparkMax blasterDrive1;
  private CANSparkMax blasterDrive2;

  /** Creates a new IntakeSubsystem. */
  public BlasterSubsystem() {
    blasterDrive1 = new CANSparkMax(Constants.blasterDrive1_ID, MotorType.kBrushless);
    blasterDrive2 = new CANSparkMax(Constants.blasterDrive2_ID, MotorType.kBrushless);

    blasterDrive1.restoreFactoryDefaults();
    blasterDrive2.restoreFactoryDefaults();

    blasterDrive2.follow(blasterDrive1, true);
  }

  public void BlasterIn() {
    blasterDrive1.set(.2);
  }

  public void BlasterAmp() {
    blasterDrive1.set(Constants.Launcher.kBlasterAmpSpeed);
  }

  public void BlasterOut() {
    blasterDrive1.set(-.6);
  }

  public void BlasterStop() {
    blasterDrive1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
