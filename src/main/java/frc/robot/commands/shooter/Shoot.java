// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  CommandSwerveDrivetrain drivetrain;
  ShooterSubsystem shooter;
  IntakeFeederSubsystem intake;
  Translation2d hubTranslation;

  public Shoot(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, IntakeFeederSubsystem intake, Translation2d hubTranslation) {
    addRequirements(drivetrain, shooter);
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.hubTranslation = hubTranslation;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var x = drivetrain.distanceToPose(hubTranslation);
    shooter.setShooterSpeed(ShooterConstants.kShooterSpeedMap.get(x));
    if (shooter.atShooterGoal()) {
      intake.setIndexer(IntakeConstants.kIndexerSpeed);
      intake.setFeeder(IntakeConstants.kFeederSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0.0);
    // shooter.setHoodGoal(0.0);
    intake.setIndexer(0.0);
    intake.setFeeder(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
