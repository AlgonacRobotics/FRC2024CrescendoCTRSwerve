// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BlasterCommands.BlasterOutSpeedUpAuto;
import frc.robot.commands.IntakeCommands.IntakeBlasterFeedAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final CommandXboxController m_operatorStick = new CommandXboxController(1); // My joystick

  // Subsystems
  public static final ElbowSubsystem m_elbowSubsystem = new ElbowSubsystem();
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static final BlasterSubsystem m_blasterSubsystem = new BlasterSubsystem();

  // commands
  private final BlasterOutSpeedUpAuto m_blasterOutSpeedUpAuto = new BlasterOutSpeedUpAuto(1000);
  private final BlasterOutSpeedUpAuto m_blasterLaunch = new BlasterOutSpeedUpAuto(1000); // was 1500
  private final IntakeBlasterFeedAuto m_IntakeBlasterFeedAuto =
      new IntakeBlasterFeedAuto(1000); // was 1500

  private final BlasterOutSpeedUpAuto m_blasterOutSpeedUpAuto1 = new BlasterOutSpeedUpAuto(1000);
  private final BlasterOutSpeedUpAuto m_blasterLaunch1 = new BlasterOutSpeedUpAuto(1000);
  private final IntakeBlasterFeedAuto m_IntakeBlasterFeedAuto1 = new IntakeBlasterFeedAuto(1000);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // configure default commands
    // set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    m_elbowSubsystem.setDefaultCommand(
        new RunCommand(() -> m_elbowSubsystem.runAutomatic(), m_elbowSubsystem));

    // set the intake to stop (0 power) when no other command is running
    m_intakeSubsystem.setDefaultCommand(
        new RunCommand(() -> m_intakeSubsystem.setPower(0.0), m_intakeSubsystem));

    // configure the launcher to stop when no other command is running
    m_blasterSubsystem.setDefaultCommand(
        new RunCommand(() -> m_blasterSubsystem.BlasterStop(), m_blasterSubsystem));

    // intake out - left bumper
        m_operatorStick.leftBumper().whileTrue(
            new RunCommand(
                () -> m_intakeSubsystem.setPower(Constants.Intake.kIntakePowerOut),
                m_intakeSubsystem));

    // Blaster Amp - x button
        m_operatorStick.x()
        .whileTrue(
            Commands.parallel(
                Commands.run(m_blasterSubsystem::BlasterAmp, m_blasterSubsystem),
                new RunCommand(
                    () -> m_intakeSubsystem.setPower(Constants.Intake.kIntakePowerBlasterAmp),
                    m_intakeSubsystem)));

    // intake in - right bumper
    m_operatorStick.rightBumper()
        .whileTrue(
            new RunCommand(
                () -> m_intakeSubsystem.setPower(Constants.Intake.kIntakePowerIn),
                m_intakeSubsystem))
        .onFalse(m_intakeSubsystem.retract());

    // elbow climb position - y button
    m_operatorStick.y()
        .onTrue(
            new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kIntakePosition)));

      //driver control of elbow climb/down position - a button
    joystick.a()
        .onTrue(
            new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kIntakePosition)));

      //driver control of elbow up position - y button
    joystick.y()
        .onTrue(
            new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kScoringPosition)));

    // elbow collect - b button
    m_operatorStick.b()
        .onTrue(
            new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kIntakePosition)))
        .onFalse(
            new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kScoringPosition)));

    // blaster score - right stick button
    m_operatorStick.rightStick()
        .onTrue(
            Commands.sequence(
                m_blasterOutSpeedUpAuto,
                Commands.parallel(m_blasterLaunch, m_IntakeBlasterFeedAuto)));

    // blaster collect - left stick button
    m_operatorStick.leftStick()
        .whileTrue(
            Commands.parallel(
                Commands.run(m_blasterSubsystem::BlasterIn, m_blasterSubsystem),
                Commands.run(m_intakeSubsystem::IntakeOut, m_intakeSubsystem)));
  }

  private void configureAutoCommands(){
     // build auto path commands
  NamedCommands.registerCommand(
    "Score",
    Commands.sequence(
        m_blasterOutSpeedUpAuto1,
        Commands.parallel(m_blasterLaunch1, m_IntakeBlasterFeedAuto1)));
        
   NamedCommands.registerCommand(
    "IntakeIn",
        new RunCommand(
                () -> m_intakeSubsystem.setPower(Constants.Intake.kIntakePowerIn),
                m_intakeSubsystem));

NamedCommands.registerCommand(
    "IntakeRetract",
        new RunCommand(
                () -> m_intakeSubsystem.retract(),
                m_intakeSubsystem));

  NamedCommands.registerCommand(
    "IntakeStop",
    new RunCommand(
                () -> m_intakeSubsystem.setPower(0),
                m_intakeSubsystem));

  NamedCommands.registerCommand(
    "IntakeDown",
    new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kIntakePosition)));
 
    NamedCommands.registerCommand(
    "IntakeUp",
    new InstantCommand(
                () -> m_elbowSubsystem.setTargetPosition(Constants.Arm.kScoringPosition))); 

    //add commands to auto chooser
   // m_chooser.setDefaultOption("Do nothing", new InstantCommand());

     /************ Center Exit Start Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    //Command centerExitStartAuto = new PathPlannerAuto("CenterExitStartAuto");
    //m_chooser.addOption("Center Exit Start Auto", centerExitStartAuto);

     /************ Amp Exit Start Auto Path ************
     *
     * starts in Amp position of the speaker, then drives
     *
     */
    //Command ampExitStartAuto = new PathPlannerAuto("AmpExitStartAuto");
    //m_chooser.addOption("Amp Exit Start Auto", ampExitStartAuto);

    /************ Source Exit Start Auto Path ************
     *
     * starts in source position of the speaker, then drives
     *
     */
    Command sourceExitStartAuto = new PathPlannerAuto("SourceExitStartAuto");
    m_chooser.addOption("Source Exit Start Auto", sourceExitStartAuto);

    /************ Center Shoot Only Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    Command centerShootAuto = new PathPlannerAuto("CenterShootAuto");
    m_chooser.setDefaultOption("Center Shoot Only Auto", centerShootAuto);

     /************ Amp Shoot Only Auto Path ************
     *
     * starts in Amp position of the speaker, then drives
     *
     */
    Command ampShootAuto = new PathPlannerAuto("AmpShootAuto");
    m_chooser.addOption("Amp Shoot Only Auto", ampShootAuto);

    /************ Source Shoot Only Auto Path ************
     *
     * starts in source position of the speaker, then drives
     *
     */
    Command sourceShootAuto = new PathPlannerAuto("SourceShootAuto");
    m_chooser.addOption("Source Shoot Only Auto", sourceShootAuto);

    /************ Center Double Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    Command centerDoubleAuto = new PathPlannerAuto("CenterDoubleAuto");
    m_chooser.addOption("Center Double Auto", centerDoubleAuto);

    /************ Source Double Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    Command sourceDoubleAuto = new PathPlannerAuto("SourceDoubleAuto");
    m_chooser.addOption("Source Double Auto", sourceDoubleAuto);

    /************ Amp Double Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    Command ampDoubleAuto = new PathPlannerAuto("AmpDoubleAuto");
    m_chooser.addOption("Amp Double Auto", ampDoubleAuto);

     /************ Center Triple Auto Path ************
     *
     * The path scores in the center position of the speaker then drives
     *
     */
    Command centerTripleAuto = new PathPlannerAuto("CenterTripleAuto");
    m_chooser.addOption("Center Triple Auto", centerTripleAuto);




    
    SmartDashboard.putData("Auto choices:", m_chooser);
  }

  public RobotContainer() {
    configureBindings();
    configureAutoCommands();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
 