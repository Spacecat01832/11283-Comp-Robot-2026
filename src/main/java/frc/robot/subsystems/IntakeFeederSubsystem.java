// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeFeederSubsystem extends SubsystemBase {

  private SparkMax IntakeMotor = new SparkMax(MotorIDs.kIntake, MotorType.kBrushless);

  private SparkMax IntakeMoverMotor = new SparkMax(MotorIDs.kIntakeMover, MotorType.kBrushless);

  private ProfiledPIDController mPID = new ProfiledPIDController(
      0,
      0,
      0,
      new TrapezoidProfile.Constraints(IntakeConstants.kIntakeMoverSpeed, 0.1));

  private SparkMax feederMotor = new SparkMax(MotorIDs.kFeeder, MotorType.kBrushless);

  private SparkMax indexMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);

  public IntakeFeederSubsystem() {
    mPID.setGoal(0);
  }

  @Override
  public void periodic() {
    IntakeMoverMotor.set(mPID.calculate(IntakeMoverMotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("intakeMoverPosition", IntakeMoverMotor.getEncoder().getPosition());
  }

  public void setIntake(double speed) {
    IntakeMotor.set(speed);
  }

  public void setFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void setIndexer(double speed) {
    indexMotor.set(speed);
  }

  public void setIntakeMover(double position) {
    mPID.setGoal(position);
  }

  public double getIntakeMoverPosition() {
    return IntakeMoverMotor.getEncoder().getPosition();
  }

  public boolean isIntakeMoverAtGoal() {
    return mPID.atGoal();
  }
}
