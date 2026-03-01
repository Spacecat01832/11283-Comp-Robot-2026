// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class changeShooterSpeeds extends InstantCommand {
  TurretSubsystem turret;
  double topSpeed;
  double bottomSpeed;
  public changeShooterSpeeds(
    TurretSubsystem turret,
    double topSpeed,
    double bottomSpeed
  ) {
    addRequirements(turret);
    this.turret = turret;
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.Shooter(topSpeed, bottomSpeed);
  }
}
