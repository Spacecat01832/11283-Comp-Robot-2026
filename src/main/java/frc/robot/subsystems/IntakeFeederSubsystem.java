// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeFeederSubsystem extends SubsystemBase {

  private SparkMax IntakeMotor = new SparkMax(MotorIDs.kIntake, MotorType.kBrushless);

  private SparkMax IntakeFlopperMotor = new SparkMax(MotorIDs.kIntakeFlopper, MotorType.kBrushless);

  private ProfiledPIDController flopperPID = new ProfiledPIDController(
      0,
      0,
      0,
      new TrapezoidProfile.Constraints(IntakeConstants.kIntakeFlopperMaxSpeed, 0.1));

  private ProfiledPIDController indexerPID = new ProfiledPIDController(
      0,
      0,
      0,
      new TrapezoidProfile.Constraints(IntakeConstants.kIndexerMaxSpeed, 0.1));

  private SparkMax feederMotor = new SparkMax(MotorIDs.kFeeder, MotorType.kBrushless);

  private SparkMax indexMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);

  public IntakeFeederSubsystem() {
    flopperPID.setGoal(0);
  }

  @Override
  public void periodic() {
    IntakeFlopperMotor.set(flopperPID.calculate(IntakeFlopperMotor.getEncoder().getPosition()));
    indexMotor.set(indexerPID.calculate(indexMotor.getEncoder().getVelocity()));
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

  public void setIntakeFlopper(double position) {
    if (position < 0) {
      position = 0;
    } else if (position > IntakeConstants.koutPosition) {
      position = IntakeConstants.koutPosition;
    }
    flopperPID.setGoal(position);
  }

  public double getIntakeFlopperPosition() {
    return IntakeFlopperMotor.getEncoder().getPosition();
  }

  public boolean isIntakeFlopperAtGoal() {
    return flopperPID.atGoal();
  }
}
