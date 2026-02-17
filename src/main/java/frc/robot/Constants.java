// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond =
        DriveConstants.kMaxCapableSpeedMetersPerSecond * 0.5; // 50% of physical capability
    public static final double kMaxAngularSpeedRadiansPerSecond = (2 * Math.PI) / 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 6;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
    public static final double kPXController = 8;
    public static final double kPYController = 8;
    public static final double kPThetaController = 5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static class VisionConstants {
    // Offsets, not measured so a transform of zero
    // Positive x is forward, positive y is left, positive z is up
    // Our official configuration has, front left is camera 1, and back left is camera 2,
    // back right is camera 3, front right is camera 4
    public static final Transform3d kRobotToCam1 =
        new Transform3d(
            Distance.ofRelativeUnits(11, Units.Inches),
            Distance.ofRelativeUnits(11, Units.Inches),
            Distance.ofRelativeUnits(24, Units.Centimeters),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(30, Units.Degrees),
                Angle.ofRelativeUnits(45, Units.Degrees)));

    public static final Transform3d kRobotToCam2 =
        new Transform3d(
            Distance.ofRelativeUnits(-11, Units.Inches),
            Distance.ofRelativeUnits(11, Units.Inches),
            Distance.ofRelativeUnits(24, Units.Centimeters),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(30, Units.Degrees),
                Angle.ofRelativeUnits(135, Units.Degrees)));

    public static final Transform3d kRobotToCam3 =
        new Transform3d(
            Distance.ofRelativeUnits(-11, Units.Inches),
            Distance.ofRelativeUnits(-11, Units.Inches),
            Distance.ofRelativeUnits(24, Units.Centimeters),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(30, Units.Degrees),
                Angle.ofRelativeUnits(225, Units.Degrees)));

    public static final Transform3d kRobotToCam4 =
        new Transform3d(
            Distance.ofRelativeUnits(11, Units.Inches),
            Distance.ofRelativeUnits(-11, Units.Inches),
            Distance.ofRelativeUnits(24, Units.Centimeters),
            new Rotation3d(
                Angle.ofRelativeUnits(0, Units.Degrees),
                Angle.ofRelativeUnits(30, Units.Degrees),
                Angle.ofRelativeUnits(315, Units.Degrees)));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Pose2d autoAimTarget =
        new Pose2d(
            Distance.ofRelativeUnits(13, Units.Meters),
            Distance.ofRelativeUnits(6, Units.Meters),
            new Rotation2d());

    public static final double kAutoAimP = 0.75;
  }

  // Constants specifically for Driving & Operation
  public static class DriveConstants {
    // Controller Ports ---
    // Determined here but assigned in the driver station to determine and organize physical ports
    // the controllers are plug into
    public static final int kDrveControllerPort = 0;
    public static final int kOperControllerPort = 1;

    // Driver Controller Joystick ---
    public static final int kDriveX = 0;
    public static final int kDriveY = 1;
    public static final int kDriveRotate = 4;
    public static final double deadzoneDriver = 0.12;

    public enum joysticks {
      DRIVER,
      OPERATOR
    }

    // Physical Constants for Speed Calculation
    // Verify these match your physical robot configuration ---------------------------------------
    public static final double kWheelDiameterMeters =
        edu.wpi.first.math.util.Units.inchesToMeters(3.0);
    public static final double kDrivingMotorPinionTeeth = 13.0; // Options: 12T, 13T, or 14T
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;

    // REV MAXSwerve Module Internal Gear Ratios
    public static final double kDrivingMotorSpurGearTeeth = 22.0;
    public static final double kDrivingMotorBevelPinionTeeth = 15.0;
    public static final double kDrivingMotorWheelBevelGearTeeth = 45.0;

    // Stage 1: Motor Pinion -> Spur Gear
    // Stage 2: Bevel Pinion -> Wheel Bevel Gear
    // Ratio = (Driven / Driver) * (Driven / Driver)
    public static final double kDrivingMotorReduction =
        (kDrivingMotorSpurGearTeeth / kDrivingMotorPinionTeeth)
            * (kDrivingMotorWheelBevelGearTeeth / kDrivingMotorBevelPinionTeeth);
    // --------------------------------------------------------------------------------------------

    // Calculated Max Speed
    // Theoretical maximum speed supported by the hardware (Physical Capability)
    public static final double kMaxCapableSpeedMetersPerSecond =
        (kDrivingMotorFreeSpeedRps * (kWheelDiameterMeters * Math.PI)) / kDrivingMotorReduction;

    // Max Allowed Speed (Software Limit) - Percentage of physical capability
    public static final double kMaxSpeedMetersPerSecond = kMaxCapableSpeedMetersPerSecond * 1.0;

    public static final double kMaxAngularSpeed = 1.25 * 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 5; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth =
        Distance.ofRelativeUnits(26.5, Units.Inches).in(Units.Meters);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase =
        Distance.ofRelativeUnits(26.5, Units.Inches).in(Units.Meters);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }

  // Constants specifically for the physical robot
  public static final class RobotConstants {
    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 2;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kFrontLeftTurningCanId = 4;

    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 6;

    public static final int kRearLeftDrivingCanId = 9;
    public static final int kRearLeftTurningCanId = 8;

    public static final int kFlywheelCanId = 9;
    public static final int kIntakeWheelCanId = 10;
    public static final int kFeederCanId = 11;

    // Used to declare Navx as upside down
    public static final boolean kGyroReversed = true;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final double kFlywheelSpeed = 0.5;
    public static final double kIntakeWheelSpeed = 0.5;
    public static final double kFeederSpeed = 0.5;
  }

  // Constants specifically for Swerve Module
  public static final class ModuleConstants {
    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingEncoderPositionFactor =
        (DriveConstants.kWheelDiameterMeters * Math.PI)
            / DriveConstants.kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((DriveConstants.kWheelDiameterMeters * Math.PI) / DriveConstants.kDrivingMotorReduction)
            / 60.0; // meters per second
    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    // PID Driving Values ---
    // Most likely used to act as a form of slew
    // See:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / DriveConstants.kMaxCapableSpeedMetersPerSecond;
    public static final double kDrivingMaxOutput = 1;
    public static final double kDrivingMinOutput = kDrivingMaxOutput * -1;
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMaxOutput = 1;
    public static final double kTurningMinOutput = kTurningMaxOutput * -1;

    // Swerve Module Idle Modes ---
    // These determine how the module behavior when there is a lack of input from the driver
    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    // Current limits ---
    // Meant for the electrical side of the drivetrain to make sure that the drivetrain isn't
    // drawing too much power
    // 50 amps for driving recommended safe limit against brownouts, could go to 60 to prioritize
    // pushing power
    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
