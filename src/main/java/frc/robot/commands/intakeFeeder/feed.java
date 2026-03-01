// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeFeeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class feed extends Command {
  boolean mEnd;
  private IntakeFeederSubsystem intake;
  private int direction;
  public feed(
    IntakeFeederSubsystem intake,
    int direction
  ) {
    addRequirements(intake);
    this.intake = intake;
    this.direction = direction;
  }

  @Override
  public void initialize() {
    mEnd = false;
  }

  @Override
  public void execute() {
    intake.setfeeders(IntakeConstants.kFeederSpeed*direction);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setfeeders(0);
  }

  @Override
  public boolean isFinished() {
    if (mEnd) {
      return true;
    }
    return false;
  }
}
