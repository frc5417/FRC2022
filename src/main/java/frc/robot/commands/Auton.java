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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Auton extends SequentialCommandGroup{

  RamseteCommand ramseteCommand;
  Drivetrain drive;

  public Auton(Drivetrain drive) {
    this.drive = drive;
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.kSAuto, Constants.kVAuto, Constants.kAAuto), 
        Constants.kinematics, Constants.maxVoltage);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.maxSpeed, Constants.maxAcceleration)
          .setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint);

    Trajectory tragic = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(
        new Translation2d(1,1),
        new Translation2d(2,-1)
      ), 
      new Pose2d(3,0, new Rotation2d(0)), 
      config);

    drive.resetOdometry(tragic.getInitialPose());

    RamseteController ramseteControl = new RamseteController();

    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.kSAuto, Constants.kVAuto, Constants.kAAuto);

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

}
