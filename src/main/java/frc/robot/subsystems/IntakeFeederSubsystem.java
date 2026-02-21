// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PneumaticConstants;

public class IntakeFeederSubsystem extends SubsystemBase {

  private Solenoid intakeSolenoid = new Solenoid(
      PneumaticConstants.kPneumaticsModuleType,
      PneumaticConstants.intakeID);

  private final SparkMax intakeMotor = new SparkMax(MotorIDs.kIntake, MotorType.kBrushless);

  public IntakeFeederSubsystem() {
  }

  @Override
  public void periodic() {
  }

  public void setIntakeSolenoid(boolean value) {
    intakeSolenoid.set(value);
  }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }
}
