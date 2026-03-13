// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  CommandSwerveDrivetrain drivetrain;
  ShooterSubsystem shooter;
  IntakeFeederSubsystem intake;

  public Shoot(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, IntakeFeederSubsystem intake) {
    addRequirements(drivetrain, shooter);
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    var x = drivetrain.distanceToPose(
        DriverStation.getAlliance().get() == Alliance.Red
            ? drivetrain.pathfromfile("RedHub").getPoint(0).position
            : drivetrain.pathfromfile("BlueHub").getPoint(0).position);
    shooter.setShooterSpeed((2.6756 * (x * x) + -3.3779 * x + 61.054));
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
