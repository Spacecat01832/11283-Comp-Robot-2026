// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.MotorIDs;
// import frc.robot.Constants.PneumaticConstants;

// public class ClimberSubsystem extends SubsystemBase {

//   private ClimberConstants pids = new ClimberConstants();

//   private final SparkMax altMotor = new SparkMax(MotorIDs.kClimberAlt, MotorType.kBrushless);
//   private final SparkMax rollMotor = new SparkMax(MotorIDs.kClimberRoll, MotorType.kBrushless);

//   private DoubleSolenoid clawSolenoid = new DoubleSolenoid(
//       PneumaticConstants.kPneumaticsModuleType,
//       PneumaticConstants.clawID1,
//       PneumaticConstants.clawID2);

//   private ProfiledPIDController altPid = pids.kAltPid;
//   private ElevatorFeedforward altFeed = pids.kAltEleFeed;
//   private ProfiledPIDController rollPid = pids.kRollPid;

//   public ClimberSubsystem() {
//     altPid.setGoal(0);
//     rollPid.setGoal(0);
//     clawSolenoid.set(Value.kReverse);
//   }

//   @Override
//   public void periodic() {
//     altMotor.set(
//         altPid.calculate(
//             MathUtil.clamp(
//                 altMotor.getEncoder().getPosition()
//                     + altFeed.calculate(altPid.getSetpoint().velocity),
//                 ClimberConstants.kAltPidMin,
//                 ClimberConstants.kAltPidMax)));
//     rollMotor.set(
//         rollPid.calculate(
//             rollMotor.getEncoder().getPosition()));
//   }

//   public void setAltGoal(double distance) {
//     altPid.setGoal(distance / ClimberConstants.kAltConvertion);
//   }

//   public boolean atAltGoal() {
//     return altPid.atGoal();
//   }

//   public void setRollGoal(double degrees) {
//     rollPid.setGoal(degrees / ClimberConstants.kRollConvertion);
//   }

//   public boolean atRollGoal() {
//     return rollPid.atGoal();
//   }

//   public void setClaw(Value value) {
//     clawSolenoid.set(value);
//   }
// }