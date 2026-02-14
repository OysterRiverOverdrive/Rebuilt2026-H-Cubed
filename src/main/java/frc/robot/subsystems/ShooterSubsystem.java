// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.VisionConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax shooterMain =
      new SparkMax(RobotConstants.kShooterMainCanId, MotorType.kBrushless);

  private RelativeEncoder shooterEncoder = shooterMain.getEncoder();

  private SparkClosedLoopController shooterPID;

  private SparkMaxConfig shooterConfig = new SparkMaxConfig();

  private double speed;

  private boolean activePID;

  private double kp = 0.0001;
  private double ki = 0;
  private double kd = 0;

  private DrivetrainSubsystem drive;

  public ShooterSubsystem(DrivetrainSubsystem drive) {
    this.drive = drive;

    SmartDashboard.putNumber("shoot p", kp);
    SmartDashboard.putNumber("shoot i", ki);
    SmartDashboard.putNumber("shoot d", kd);

    shooterConfig.idleMode(IdleMode.kCoast)
    .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
    shooterConfig.encoder.positionConversionFactor(1)
    .velocityConversionFactor(1);
    
    shooterMain.configure(shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterPID = shooterMain.getClosedLoopController();
  }

  public void shooterDislodgeCmd() {
    shooterMain.set(0.2);
  }

  public void shooterShootCmd() {
    activePID = true;
  }

  public void shooterStopCmd() {
    activePID = false;
    shooterPID.setSetpoint(0, ControlType.kVelocity);
    shooterMain.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getVelocity());
    
    double distance = Math.abs(Math.hypot(VisionConstants.autoAimTarget.getY() - drive.getVisionPose().getY(),
              VisionConstants.autoAimTarget.getX() - drive.getVisionPose().getX()));
    SmartDashboard.putNumber("Distance to Hub", distance);
    if (distance < 650) {
      SmartDashboard.putBoolean("Can Shoot", true);
      speed = (14.70954 * distance) + 6776.07278;
    } else {
      SmartDashboard.putBoolean("Can Shoot", false);
      speed = (14.70954 * 650) + 6776.07278;
    }

    if (activePID) {
      shooterPID.setSetpoint(-1 * speed, ControlType.kVelocity);
      SmartDashboard.putNumber("Shooting Setpoint", speed);
    }
  }
}
