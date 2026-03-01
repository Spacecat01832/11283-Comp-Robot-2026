// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

  private TurretConstants pids = new TurretConstants();

  private final TalonFX bottomMotor = new TalonFX(MotorIDs.kBottemTurret);
  private final SparkMax topMotor = new SparkMax(MotorIDs.kTopTurret, MotorType.kBrushless);
  private final SparkMax yawMotor = new SparkMax(MotorIDs.kTurretYaw, MotorType.kBrushless);

  private ProfiledPIDController yawPid = pids.kPid;

  private SimpleMotorFeedforward yawFeed = pids.kFeedFor;

  private PIDController topRevLimiter = pids.topRevLimiter;
  private PIDController bottomRevLimiter = pids.bottomRevLimiter;

  public TurretSubsystem() {
    yawPid.setGoal(0);
    yawPid.setTolerance(TurretConstants.kPidDeadband);
    topRevLimiter.setSetpoint(0);
    bottomRevLimiter.setSetpoint(0);
  }

  @Override
  public void periodic() {
    yawMotor.set(yawPid.calculate(yawMotor.getEncoder().getPosition() + yawFeed.calculate(yawPid.getSetpoint().velocity)));
    SmartDashboard.putNumber("tyaw", yawMotor.getEncoder().getPosition());
    topMotor.set(topRevLimiter.calculate(topMotor.getEncoder().getVelocity()));
    bottomMotor.set(bottomRevLimiter.calculate(bottomMotor.getVelocity().getValueAsDouble()));
  }

  public void setYawGoal(double degrees) {
    yawPid.setGoal(degrees / TurretConstants.kConvertion);
  }

  public boolean atYawGoal() {
    return yawPid.atGoal();
  }

  public void Shooter(double topSpeed, double bottomSpeed) {
    topRevLimiter.setSetpoint(-topSpeed);
    bottomRevLimiter.setSetpoint(-bottomSpeed);
  }
}
