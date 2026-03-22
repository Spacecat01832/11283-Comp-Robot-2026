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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.*;
import frc.robot.commands.intakeFeeder.*;
import frc.robot.commands.shooter.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

  final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // final CommandXboxController buttonController = new
  // CommandXboxController(OperatorConstants.kButtonControllerPort);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.kMaxSpeed * 0.2).withRotationalDeadband(DriveConstants.kMaxAngularRate * 0.2)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeFeederSubsystem intakeFeeder = new IntakeFeederSubsystem();

  private final SetIntakePosition setIntakePosition(double position) {
    return new SetIntakePosition(intakeFeeder, position);
  }

  private final Shoot shoot;

  private boolean isRed = false;

  private Translation2d hubTranslation = new Translation2d(0, 0);

  public SendableChooser<Command> AutoChooser;

  public NetworkTable tbl = NetworkTableInstance.getDefault().getTable("robot");

  public NetworkTableEntry shooterspeed;

  public RobotContainer() {
    // Determine alliance and hub translation first so commands that depend on
    // them can be constructed before binding buttons.
  isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    hubTranslation = isRed ? drivetrain.pathfromfile("RedHub").getPoint(0).position
        : drivetrain.pathfromfile("BlueHub").getPoint(0).position;

    // Construct the shoot command now so it can be registered and bound.
    shoot = new Shoot(drivetrain, shooter, intakeFeeder, hubTranslation);

    // Register commands and configure button bindings.
    registerCommands();
    configureBindings();

    // Build the auto chooser (uses any registered NamedCommands)
    AutoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", AutoChooser);

    shooterspeed = tbl.getEntry("shooterspeed");
    shooterspeed.setDouble(0);
  }

  private void registerCommands(){
    NamedCommands.registerCommand("IntakeOut", setIntakePosition(IntakeConstants.koutPosition));
    NamedCommands.registerCommand("IntakeIn", setIntakePosition(0));
    NamedCommands.registerCommand("shoot", shoot);
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

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(
        drivetrain.applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    driverController.leftBumper()
        .onTrue(setIntakePosition(IntakeConstants.koutPosition))
        .onFalse(setIntakePosition(0));

    driverController.rightTrigger(0.3).onTrue(
        Commands.run(() -> {
          //var x = drivetrain.distanceToPose(hubTranslation);
          shooter.setShooterSpeed(shooterspeed.getDouble(0));
          intakeFeeder.setFeeder(IntakeConstants.kFeederSpeed);
          intakeFeeder.setIndexer(IntakeConstants.kIndexerSpeed);
        }, shooter, intakeFeeder)).onFalse(
            Commands.run(() -> {
              intakeFeeder.setFeeder(0);
              intakeFeeder.setIndexer(0);
              shooter.setShooterSpeed(0);
            }, shooter, intakeFeeder));

    driverController.rightBumper().onTrue(
        shoot)
        .onFalse(
            Commands.runOnce(() -> {
              shoot.cancel();
            }, shooter, intakeFeeder, drivetrain));
  }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
  }
}