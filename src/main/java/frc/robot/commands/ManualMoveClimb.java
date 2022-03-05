// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ManualMoveClimb extends CommandBase {
  private final Climber climb;
  private final RobotContainer robotContainer;
  
  /** Creates a new ManualMoveClimb. */
  public ManualMoveClimb(Climber climb, RobotContainer robotContainer) {
    this.climb = climb;
    this.robotContainer = robotContainer;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled. 
  @Override
  public void execute() {
    climb.setPower(robotContainer.getManipulatorPad().getRawAxis(1), robotContainer.getManipulatorPad().getRawAxis(5));
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
