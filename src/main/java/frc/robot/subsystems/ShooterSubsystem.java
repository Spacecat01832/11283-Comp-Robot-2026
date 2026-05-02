// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorIDs;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(MotorIDs.kShooter);
  private double shooterSpeed;

  private PIDController ShooterLimiter = new PIDController(0.004,0.0002,0.0002);

  private SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(0.011, 0.01);

  // private PIDController ShooterLimiter = new PIDController(0.1794, 0, 0);

  // private SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(0.10183, 0.16698, 0.011862);

  public ShooterSubsystem() {
    shooterSpeed = 0;
    ShooterLimiter.setTolerance(0.4);
    ShooterLimiter.setSetpoint(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("speed", shooterMotor.getVelocity().getValueAsDouble());
    if (shooterSpeed == 0) {
      shooterMotor.set(0);
    } else {
      shooterMotor.set(
          ShooterLimiter.calculate(shooterMotor.getVelocity().getValueAsDouble())
              + shooterFeed.calculate(ShooterLimiter.getSetpoint()));
      if (shooterMotor.getVelocity().getValueAsDouble() < shooterSpeed - 3) {
        ShooterLimiter.setSetpoint(shooterSpeed + 5);
      }
    }
  }

  public void setShooterSpeed(double speed) {
    shooterSpeed = speed;
  }

  public boolean atShooterGoal() {
    return ShooterLimiter.atSetpoint();
  }
}
