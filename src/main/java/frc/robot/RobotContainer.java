// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.intakeFeeder.*;
import frc.robot.commands.shooter.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

  final CommandPS5Controller driverController = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  // final CommandXboxController buttonController = new
  // CommandXboxController(OperatorConstants.kButtonControllerPort);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.kMaxSpeed * 0.1).withRotationalDeadband(DriveConstants.kMaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IntakeFeederSubsystem intakeFeeder = new IntakeFeederSubsystem();

  private final SetIntakePosition setIntakePosition(double position) {
    return new SetIntakePosition(intakeFeeder, position);
  }

  private final SetIntakeSpeed setIntakeSpeed(double speed) {
    return new SetIntakeSpeed(intakeFeeder, speed);
  }

  private final Shoot shoot(Translation2d hub) {
    return new Shoot(drivetrain, shooter, intakeFeeder, hub);
  }

  public Translation2d hubposition = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
      ? drivetrain.pathfromfile("RedHub").getAllPathPoints().get(0).position
      : drivetrain.pathfromfile("BlueHub").getAllPathPoints().get(0).position;

  public SendableChooser<Command> AutoChooser;

  public RobotContainer() {
    registerCommands();
    configureBindings();
    AutoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", AutoChooser);
  }

  private void registerCommands() {
    NamedCommands.registerCommand("IntakeOut", setIntakePosition(IntakeConstants.koutPosition));
    NamedCommands.registerCommand("IntakeIn", setIntakePosition(0.0));
    NamedCommands.registerCommand("Intake", setIntakeSpeed(IntakeConstants.kIntakeSpeed));
    NamedCommands.registerCommand("Intakeoff", setIntakeSpeed(0));
    NamedCommands.registerCommand("shoot", shoot(hubposition));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * DriveConstants.kMaxSpeed)
            .withVelocityY(-driverController.getLeftX() * DriveConstants.kMaxSpeed)
            .withRotationalRate(-driverController.getRightX() * DriveConstants.kMaxAngularRate)));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    driverController.cross().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.circle().whileTrue(
        drivetrain.applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverController.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    driverController.L1()
        .onTrue(setIntakePosition(0.0))
        .onFalse(setIntakePosition(IntakeConstants.koutPosition));

    driverController.L2()
        .onTrue(
            setIntakeSpeed(IntakeConstants.kIntakeSpeed))
        .onFalse(
            setIntakeSpeed(0.0));

    driverController.R2()
        .onTrue(Commands.run(() -> {
          var timer = new Timer();
          timer.start();
          timer.reset();
          shooter.setShooterSpeed(60);
          if (timer.get() > 0.8) {
            intakeFeeder.setIndexer(IntakeConstants.kIndexerSpeed);
            intakeFeeder.setFeeder(IntakeConstants.kFeederSpeed);
          }
        }, shooter, intakeFeeder))
        .onFalse(Commands.run(() -> {
          shooter.setShooterSpeed(0);
          intakeFeeder.setIndexer(0);
          intakeFeeder.setFeeder(0);
        }, shooter, intakeFeeder));

    driverController.R1()
        .whileTrue(
            shoot(hubposition));
  }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
  }
}