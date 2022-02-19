// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

public class Auton2 extends SequentialCommandGroup{

  RamseteCommand ramseteCommand;
  Drive drive;

  public Auton2(Drive drive) {
    this.drive = drive;
    SimpleMotorFeedforward motorFF = new SimpleMotorFeedforward(Constants.kSAuto, Constants.kVAuto, Constants.kAAuto);

    /*note- it seems like the trajectory is always inverted:
      When you tell it to go forwards, it goes backwards; vice versa
      When you tell it to turn right, it turns left; vice versa
      Whenever you tell it to turn in an arc, it goes along the arc for half of the distance and then reverses the direction and tries to go back- has an error
      Relative distance- 1.0 unit on PathPlanner is roughly 1 foot
    */
    Trajectory tragic = PathPlanner.loadPath("TurnTest3", Constants.maxSpeed, Constants.maxAcceleration);

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

}
