// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.Turret.changeShooterSpeeds;
import frc.robot.commands.intakeFeeder.*;
import frc.robot.generated.TunerConstants;
// import frc.robot.commands.Climber.climbSetup;
// import frc.robot.commands.Climber.setAltPosition;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Auto.AutoSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {

  final AutoSubsystem auto = new AutoSubsystem();

  final TurretSubsystem turret = new TurretSubsystem();
  final IntakeFeederSubsystem intakeFeeder = new IntakeFeederSubsystem();
  // final ClimberSubsystem climber = new ClimberSubsystem();

  // final climbSetup climbSetup = new climbSetup(climber);
  final Command intake(int direction) {
    return new intake(intakeFeeder, direction);
  }

  final Command feed(int direction) {
    return new feed(intakeFeeder, direction);
  }

  final Command moveIntake() {
    return new moveIntake(intakeFeeder);
  }

  final Command setShooter(double topspeed, double bottomSpeed){
    return new changeShooterSpeeds(turret, topspeed, bottomSpeed);
  }

  final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  final CommandXboxController buttonController = new CommandXboxController(
      OperatorConstants.k2ndDriverControllerPort);

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * 0.7 * MaxSpeed) // Drive forward with negative Y
                                                                                       // (forward)
            .withVelocityY(-joystick.getLeftX() * 0.8 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * 1.55 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(
        drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // buttonController.y()
    // .whileTrue(climbSetup)
    // .whileFalse(new setAltPosition(climber, 0));

    buttonController.back()
        .onTrue(moveIntake());
    buttonController.a()
        .whileTrue(intake(1));
    buttonController.b()
        .whileTrue(intake(-1));
    buttonController.rightBumper()
        .whileTrue(setShooter(-500, 213))
        .onFalse(setShooter(0, 0));
    buttonController.leftBumper()
        .whileTrue(setShooter(-1000, 426))
        .onFalse(setShooter(0, 0));
    buttonController.y()
        .whileTrue(feed(1));
  }

  public Command getAutonomousCommand() {
    return auto.getSelectedAuto();
  }

  public void periodic() {
  }
}