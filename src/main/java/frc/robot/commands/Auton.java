// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;
public class Auton extends SequentialCommandGroup{

  RamseteCommand ramseteCommand;
  Drive drive;

  public Auton(Drive drive) {
    this.drive = drive;
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.kSAuto, Constants.kVAuto, Constants.kAAuto);
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        motorFF, 
        Constants.kinematics, Constants.maxVoltage);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.maxSpeed, Constants.maxAcceleration)
          .setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint);

    Trajectory tragic = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(
        new Translation2d(2,-1),
        new Translation2d(3,0),
        new Translation2d(4,1)
      ), 
      new Pose2d(6,0, new Rotation2d(0)), 
      config);

    drive.resetOdometry(tragic.getInitialPose());

    RamseteController ramseteControl = new RamseteController();

    ramseteCommand = new RamseteCommand(
      tragic, 
      drive::getPose,
      ramseteControl, 
      motorFF, 
      Constants.kinematics, 
      drive::getWheelSpeeds, 
      new PIDController(Constants.kPAuto, 0, 0), 
      new PIDController(Constants.kPAuto, 0, 0), 
      drive::tankDriveVolts, 
      drive);
  }

  public Command getRamseteCommand (){
    return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
  }

//   @Override
//   public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
//                                             double velocityMetersPerSecond) {
//   return 10;
// }

//   @Override
//   public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters,
//                                                      double curvatureRadPerMeter,
//                                                      double velocityMetersPerSecond) {
//   return new MinMax(0, 3);
// }

}
