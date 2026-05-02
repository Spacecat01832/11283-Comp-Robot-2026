// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeFeederSubsystem extends SubsystemBase {

  private TalonFX IntakeMotor = new TalonFX(MotorIDs.kIntake);
  private double intakespeed;

  private SparkMax IntakeFlopperMotor = new SparkMax(MotorIDs.kIntakeFlopper, MotorType.kBrushless);
  // private boolean floppersetpoint;

  private SparkMax feederMotor = new SparkMax(MotorIDs.kFeeder, MotorType.kBrushless);
  private double feedspeed;

  private SparkMax indexMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);
  private double indexspeed;

  private SparkClosedLoopController flopperPID = IntakeFlopperMotor.getClosedLoopController();

  public IntakeFeederSubsystem() {
    flopperPID.setSetpoint(0, ControlType.kPosition);
    feedspeed = 0;
    indexspeed = 0;
    intakespeed = 0;
  }

  @Override
  public void periodic() {
    // IntakeFlopperMotor.set(
    // floppersetpoint
    // ? IntakeFlopperMotor.getEncoder().getPosition() >
    // IntakeConstants.koutPosition + 0.1
    // ? -IntakeConstants.kIntakePoSpeed + 0.25
    // : 0
    // : IntakeFlopperMotor.getEncoder().getPosition() < -0.1
    // ? 1
    // : 0);
    IntakeMotor.set(intakespeed);
    feederMotor.set(feedspeed);
    indexMotor.set(indexspeed);
  }

  public void setFeeder(double speed) {
    feedspeed = speed;
  }

  public void setIntake(double speed) {
    intakespeed = speed;
  }

  public void setIndexer(double speed) {
    indexspeed = speed;
  }

  public void setIntakeFlopper(double position) {
    flopperPID.setSetpoint(position > 0
        ? 0
        : position < IntakeConstants.koutPosition
            ? IntakeConstants.koutPosition
            : position,
        ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // public void setIntakeFlopper(boolean out) {
  // floppersetpoint = out;
  // }

  public double getIntakeFlopperPosition() {
    return IntakeFlopperMotor.getEncoder().getPosition();
  }

  public boolean isIntakeFlopperAtGoal() {
    return flopperPID.isAtSetpoint();
  }
}
