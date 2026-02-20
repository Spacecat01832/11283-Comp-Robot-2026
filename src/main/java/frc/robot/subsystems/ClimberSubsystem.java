// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PneumaticConstants;

public class ClimberSubsystem extends SubsystemBase {

  private ClimberConstants pids = new ClimberConstants();

  private final SparkMax altMotor = new SparkMax(MotorIDs.kClimberAlt, MotorType.kBrushless);
  private final SparkMax rollMotor = new SparkMax(MotorIDs.kClimberRoll, MotorType.kBrushless);

  private final DoubleSolenoid clawSolenoid = new DoubleSolenoid(
      PneumaticConstants.kPneumaticsModuleType,
      PneumaticConstants.kClimberID1,
      PneumaticConstants.kClimberID2);

  private ProfiledPIDController elePid = pids.kAltPid;
  private ElevatorFeedforward eleFeed = pids.kAltEleFeed;
  private ProfiledPIDController rollPid = pids.kRollPid;

  public ClimberSubsystem() {
    elePid.setGoal(0);
    rollPid.setGoal(0);
  }

  @Override
  public void periodic() {
    altMotor.set(
        elePid.calculate(
            MathUtil.clamp(
                altMotor.getEncoder().getPosition()
                    + eleFeed.calculate(elePid.getSetpoint().velocity),
                ClimberConstants.kAltPidMin,
                ClimberConstants.kAltPidMax)));
    rollMotor.set(
        rollPid.calculate(
            rollMotor.getEncoder().getPosition()));
  }

  public void setAltGoal(double distance) {
    elePid.setGoal(distance/ClimberConstants.kAltConvertion);
  }

  public void setRollGoal(double degrees) {
    rollPid.setGoal(degrees/ClimberConstants.kRollConvertion);
  }

  public void setClaw(Value value) {
    clawSolenoid.set(value);
  }
}