// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbAnchor extends CommandBase {
  private final Climber climber;
  private final RobotContainer robotContainer;
  private boolean currentState;
  private boolean commandState;

  /** Creates a new AutoClimb. */
  public ClimbAnchor(Climber climber, RobotContainer robotContainer) {
    this.climber = climber;
    this.robotContainer = robotContainer;
    currentState = true;
    commandState = false;
    //this.anchor = this.climber.getAnchorSolenoid();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (currentState == true){
      this.climber.setPassiveClamp(true);
      currentState = false;
      commandState = true;
    }
    else if (currentState == false){
      this.climber.setPassiveClamp(false);
      currentState = true;
      commandState = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPassiveClamp(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!robotContainer.getButtonB8() && commandState){
      commandState = false;
      return true;
    }
    return false;
  }
}
