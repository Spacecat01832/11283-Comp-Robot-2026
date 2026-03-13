// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InBoxAuto extends SequentialCommandGroup {
  /** Creates a new InBoxAuto. */
  public InBoxAuto(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooter, IntakeFeederSubsystem intakeFeeder) {
    new Rotation2d();
    addCommands(
        drivetrain.resetposefrompath("alintrenchtoshooting0"),
        drivetrain.followpath("alintrenchtoshooting0"),
        Commands.race(
          new Shoot(drivetrain, shooter, intakeFeeder),
          new WaitCommand(1)
        ),
        drivetrain.followpath("shooting0toinbox")
    );
  }
}
