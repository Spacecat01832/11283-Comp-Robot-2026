// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("IntakeFeeder");

  private NetworkTableEntry intakeFlopperPositionEntry = nt.getEntry("Intake Flopper Position"),
      intakeFlopperGoalEntry = nt.getEntry("Intake Flopper Goal"),
      intakeFlopperkp = nt.getEntry("intakeFlopperkp"), 
      intakeFlopperki = nt.getEntry("intakeFlopperki"), 
      intakeFlopperkd = nt.getEntry("intakeFlopperkd"), 
      intakeFlopperMaxSpeed = nt.getEntry("intakeFlopperMaxSpeed"),
      intakeFlopperacel = nt.getEntry("intakeFlopperacel"), 
      indexerSpeedEntry = nt.getEntry("Indexer Speed"),
      indexerGoalEntry = nt.getEntry("Indexer Goal"),
      indexerkp = nt.getEntry("indexerkp"),
      indexerki = nt.getEntry("indexerki"),
      indexerkd = nt.getEntry("indexerkd"),
      indexerMaxSpeed = nt.getEntry("indexerMaxSpeed"),
      indexeracel = nt.getEntry("indexeracel");

  public IntakeFeederSubsystem() {
    flopperPID.setGoal(0);
    //setNetworkTableDefaults();
  }

  private void setNetworkTableDefaults() {
    intakeFlopperkp.setDefaultDouble(0);
    intakeFlopperki.setDefaultDouble(0);
    intakeFlopperkd.setDefaultDouble(0);
    intakeFlopperMaxSpeed.setDefaultDouble(0);
    intakeFlopperacel.setDefaultDouble(0);
    indexerkp.setDefaultDouble(0);
    indexerki.setDefaultDouble(0);
    indexerkd.setDefaultDouble(0);
    indexerMaxSpeed.setDefaultDouble(0);
    indexeracel.setDefaultDouble(0);
  }

  private void setThingsOffOfNetworkTable() {
    flopperPID.setPID(intakeFlopperkp.getDouble(0),
        intakeFlopperki.getDouble(0),
        intakeFlopperkd.getDouble(0));
    flopperPID.setConstraints(
        new TrapezoidProfile.Constraints(intakeFlopperMaxSpeed.getDouble(0), intakeFlopperacel.getDouble(0)));

    indexerPID.setPID(indexerkp.getDouble(0), indexerki.getDouble(0), indexerkd.getDouble(0));
    indexerPID.setConstraints(new TrapezoidProfile.Constraints(indexerMaxSpeed.getDouble(0), indexeracel.getDouble(0)));

    intakeFlopperPositionEntry.setDouble(IntakeFlopperMotor.getEncoder().getPosition());
    intakeFlopperGoalEntry.setDouble(flopperPID.getSetpoint().position);
    indexerGoalEntry.setDouble(indexerPID.getSetpoint().position);
    indexerSpeedEntry.setDouble(indexMotor.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    //setThingsOffOfNetworkTable();
    IntakeFlopperMotor.set(flopperPID.calculate(IntakeFlopperMotor.getEncoder().getPosition()));
    indexMotor.set(indexerPID.calculate(indexMotor.getEncoder().getVelocity()));
    IntakeMotor.set(IntakeFlopperMotor.getEncoder().getPosition() > 1 ? IntakeConstants.kIntakeSpeed : 0.0);
  }

  public void setFeeder(double speed) {
    feederMotor.set(speed);
  }

  public void setIndexer(double speed) {
    indexerPID.setGoal(speed);
  }

  public void setIntakeFlopper(double position) {
    flopperPID.setGoal(position < 0
        ? 0
        : position > IntakeConstants.koutPosition
            ? IntakeConstants.koutPosition
            : position);
  }

  public double getIntakeFlopperPosition() {
    return IntakeFlopperMotor.getEncoder().getPosition();
  }

  public boolean isIntakeFlopperAtGoal() {
    return flopperPID.atGoal();
  }
}
