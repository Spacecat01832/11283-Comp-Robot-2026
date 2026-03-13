// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAAuto extends SequentialCommandGroup {
  /** Creates a new InBoxAuto. */
  public ShootAAuto(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, IntakeFeederSubsystem intakeFeeder) {
    addCommands(
        AutoBuilder.resetOdom(drivetrain.pathfromfile("attosa").getPathPoses().get(0)),
        AutoBuilder.followPath(drivetrain.pathfromfile("attosa")),
        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new Shoot(drivetrain, shooter, intakeFeeder)));
  }
}
