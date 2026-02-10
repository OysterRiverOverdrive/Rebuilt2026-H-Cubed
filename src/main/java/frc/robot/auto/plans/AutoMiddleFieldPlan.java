// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMiddleFieldPlan extends ParallelCommandGroup {
  public AutoMiddleFieldPlan(DrivetrainSubsystem drive) {
    Command toMid = 
      AutoCreationCmd.AutoRobotDriveCmd(
        drive,
        List.of(new Translation2d(4.125, 0)),
        new Pose2d(8.25, 16.54, new Rotation2d(-Math.PI / 2)));
    addCommands(
      toMid
    );
  }
}
