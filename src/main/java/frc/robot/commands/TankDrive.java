/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class TankDrive extends CommandBase {
  private final Drive drive;
  private final RobotContainer robotContainer;
  /**
   * Creates a new TankDrive
   */
  public TankDrive(RobotContainer container, Drive subsystem) {
    drive = subsystem;
    robotContainer = container;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this sets the power of the motors to whatever is indicated by leftSpeed() and rightSpeed() from the robotContainer object
    drive.setPower(robotContainer.getDriverLeftJoystick(), robotContainer.getDriverRightJoystick());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}