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
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(MotorIDs.kShooter);
  private final SparkMax hoodMotor = new SparkMax(MotorIDs.kShooterHood, MotorType.kBrushless);

  private ProfiledPIDController hoodPid = new ProfiledPIDController(0, 0, 0, null);

  private SimpleMotorFeedforward hoodFeed = new SimpleMotorFeedforward(0, 0);

  private PIDController revLimiter = new PIDController(0, 0, 0);

  public ShooterSubsystem() {
    hoodPid.setGoal(0);
    hoodPid.setTolerance(ShooterConstants.kPidDeadband);
    revLimiter.setSetpoint(0);
  }

  @Override
  public void periodic() {
    hoodMotor.set(hoodPid.calculate(hoodMotor.getEncoder().getPosition() + hoodFeed.calculate(hoodPid.getSetpoint().velocity)));
    SmartDashboard.putNumber("tyaw", hoodMotor.getEncoder().getPosition());
    shooterMotor.set(revLimiter.calculate(shooterMotor.getVelocity().getValueAsDouble()));
  }

  public void setHoodGoal(double angle) {
    hoodPid.setGoal(angle);
  }

  public boolean atHoodGoal() {
    return hoodPid.atGoal();
  }

  public void Shooter(double speed) {
    revLimiter.setSetpoint(speed);
  }
}
