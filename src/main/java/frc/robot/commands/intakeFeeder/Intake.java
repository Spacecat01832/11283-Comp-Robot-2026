// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeFeeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class Intake extends Command {
  boolean mEnd;
  private IntakeFeederSubsystem intake;
  private boolean doIntakeReturn;
  private int intakeDirection;
  // set intake direction to 0 if you want no intake

  public Intake(
      IntakeFeederSubsystem intake,
      boolean doIntakeReturn,
      int intakeDirection) {
    addRequirements(intake);
    this.intake = intake;
    this.doIntakeReturn = doIntakeReturn;
    this.intakeDirection = MathUtil.clamp(intakeDirection, -1, 1);
  }

  @Override
  public void initialize() {
    mEnd = false;
  }

  @Override
  public void execute() {
    intake.setIntakeSolenoid(true);
    intake.setIntakeMotor(intakeDirection * IntakeConstants.kIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeMotor(0);
    if (doIntakeReturn) {
      intake.setIntakeSolenoid(false);
    }
  }

  @Override
  public boolean isFinished() {
    if (mEnd) {
      return true;
    }
    return false;
  }
}
