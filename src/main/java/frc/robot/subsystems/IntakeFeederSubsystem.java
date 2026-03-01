// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PneumaticConstants;

public class IntakeFeederSubsystem extends SubsystemBase {
  private Value value;
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
    PneumaticConstants.kPneumaticsModuleType, 
    PneumaticConstants.intakeID1, 
    PneumaticConstants.intakeID2);

  private final SparkMax intakeMotor = new SparkMax(MotorIDs.kIntake, MotorType.kBrushless);
  private final SparkMax feederMotor1 = new SparkMax(MotorIDs.kFeeder1, MotorType.kBrushless);
  private final SparkMax feederMotor2 = new SparkMax(MotorIDs.kFeeder2, MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);


  public IntakeFeederSubsystem() {
    value = Value.kReverse;
  }

  @Override
  public void periodic() {
    intakeSolenoid.set(value);
  }

  public void setIntakeSolenoid(Value value) {
    this.value = value;
  }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  public void setfeeders(double speed) {
    feederMotor1.set(speed);
    feederMotor2.set(speed*0.9);
    indexerMotor.set(speed*0.75);
  }

  public boolean currentPosition() {
    if (value == Value.kForward) {
      return true;
    } else {
      return false;
    }
  }
}
