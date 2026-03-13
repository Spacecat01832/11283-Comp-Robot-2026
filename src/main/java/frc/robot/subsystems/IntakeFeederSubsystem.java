// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeFeederSubsystem extends SubsystemBase {

  private SparkMax IntakeMotor = new SparkMax(MotorIDs.kIntake, MotorType.kBrushless);

  private SparkMax IntakeFlopperMotor = new SparkMax(MotorIDs.kIntakeFlopper, MotorType.kBrushless);

  private SparkMax feederMotor = new SparkMax(MotorIDs.kFeeder, MotorType.kBrushless);
  private double feedspeed;

  private SparkMax indexMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);

  private SparkClosedLoopController flopperPID = IntakeFlopperMotor.getClosedLoopController();

  private SparkClosedLoopController indexerPID = indexMotor.getClosedLoopController();

  public IntakeFeederSubsystem() {
    feedspeed = 0;
  }

  @Override
  public void periodic() {
    IntakeMotor.set(IntakeFlopperMotor.getEncoder().getPosition() < -1 ? IntakeConstants.kIntakeSpeed : 0.0);
    feederMotor.set(feedspeed);
  }

  public void setFeeder(double speed) {
    feedspeed = speed;
  }

  public void setIndexer(double speed) {
    indexerPID.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void setIntakeFlopper(double position) {
    flopperPID.setSetpoint(position > 0
        ? 0
        : position < IntakeConstants.koutPosition
            ? IntakeConstants.koutPosition
            : position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public double getIntakeFlopperPosition() {
    return IntakeFlopperMotor.getEncoder().getPosition();
  }

  public boolean isIntakeFlopperAtGoal() {
    return flopperPID.isAtSetpoint();
  }
}
