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

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");

  private NetworkTableEntry intakeFlopperPositionEntry = nt.getEntry("Intake Flopper Position"),
      intakeFlopperGoalEntry = nt.getEntry("Intake Flopper Goal"),
      intakeFlopperkp = nt.getEntry("intakeFlopperkp"), // TODO doesn't need to be here
      intakeFlopperki = nt.getEntry("intakeFlopperki"), // TODO doesn't need to be here
      intakeFlopperkd = nt.getEntry("intakeFlopperkd"), // TODO doesn't need to be here
      intakeFlopperMaxSpeed = nt.getEntry("intakeFlopperMaxSpeed"), // TODO doesn't need to be here
      intakeFlopperacel = nt.getEntry("intakeFlopperacel"), // TODO doesn't need to be here
      indexerSpeedEntry = nt.getEntry("Indexer Speed"),
      indexerGoalEntry = nt.getEntry("Indexer Goal"),
      indexerkp = nt.getEntry("indexerkp"), // TODO doesn't need to be here
      indexerki = nt.getEntry("indexerki"), // TODO doesn't need to be here
      indexerkd = nt.getEntry("indexerkd"), // TODO doesn't need to be here
      indexerMaxSpeed = nt.getEntry("indexerMaxSpeed"), // TODO doesn't need to be here
      indexeracel = nt.getEntry("indexeracel"); // TODO doesn't need to be here


  public IntakeFeederSubsystem() {
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

    flopperPID.setGoal(0);
  }

  @Override
  public void periodic() {
    flopperPID.setPID(intakeFlopperkp.getDouble(0),
        intakeFlopperki.getDouble(0),
        intakeFlopperkd.getDouble(0));
    flopperPID.setConstraints(new TrapezoidProfile.Constraints(intakeFlopperMaxSpeed.getDouble(0), intakeFlopperacel.getDouble(0)));

    indexerPID.setPID(indexerkp.getDouble(0), indexerki.getDouble(0), indexerkd.getDouble(0));
    indexerPID.setConstraints(new TrapezoidProfile.Constraints(indexerMaxSpeed.getDouble(0), indexeracel.getDouble(0)));

    intakeFlopperPositionEntry.setDouble(IntakeFlopperMotor.getEncoder().getPosition());
    intakeFlopperGoalEntry.setDouble(flopperPID.getSetpoint().position);
    indexerGoalEntry.setDouble(indexerPID.getSetpoint().position);
    indexerSpeedEntry.setDouble(indexMotor.getEncoder().getVelocity());

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
