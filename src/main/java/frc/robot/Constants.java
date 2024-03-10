// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>Subsystem-specific constants should be defined in the subsystem's own constant class.
 * Constants that vary from robot to robot should be defined in the config classes.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int elbowDrive_ID = 30;
  public static final int intakeDrive_ID = 31;
  public static final int blasterDrive1_ID = 32;
  public static final int blasterDrive2_ID = 33;

  public static final class Arm {
    public static final int kArmCanId = 2;
    public static final boolean kArmInverted = true;
    public static final int kCurrentLimit = 40;

    public static final double kSoftLimitReverse = -2.2;
    public static final double kSoftLimitForward = 0.0;

    public static final double kArmGearRatio = (1.0 / 25.0) * (28.0 / 50.0) * (16.0 / 64.0);
    public static final double kPositionFactor =
        kArmGearRatio
            * 2.0
            * Math.PI; // multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset =
        1.342; // radians to add to converted arm position to get real-world arm position (starts at
    // ~76.9deg angle)
    public static final ArmFeedforward kArmFeedforward =
        new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
    // public static final PIDGains kArmPositionGains = new PIDGains(2.5, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint =
        new TrapezoidProfile.Constraints(1.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 0.0;
    public static final double kIntakePosition = -2.2; // was -1.17
  }

  public static final class Intake {
    public static final int kCanId = 1;
    public static final boolean kMotorInverted = true;
    public static final int kCurrentLimit = 80;

    // public static final PIDGains kPositionGains = new PIDGains(1.0, 0.0, 0.0);
    public static final double kPositionTolerance = 0.25;

    public static final double kIntakePowerIn = 0.4;
    public static final double kIntakePowerOut = -0.4;
    public static final double kIntakePowerBlast = 0.6;
    public static final double kIntakePowerBlasterAmp = 0.4;

    public static final double kRetractDistance = -0.5;

    public static final double kShotFeedTime = 1.0;
  }

  public static final class Launcher {
    public static final int kTopCanId = 3;
    public static final int kBottomCanId = 4;

    public static final int kCurrentLimit = 80;

    public static final double kTopPower = 0.7;
    public static final double kBottomPower = 0.8;

    public static final double kBlasterAmpSpeed = 0.4;
  }
}
