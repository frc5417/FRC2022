// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.RobotContainer;

public class ClimbPivot extends CommandBase {
  private final Climber climber;
  private RobotContainer robotContainer;
  private boolean currentState;
  private boolean commandState;

  public ClimbPivot(Climber climber, RobotContainer container) {
    this.climber = climber;
    this.robotContainer = container;
    currentState = true;
    commandState = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (currentState == true){
      this.climber.setActiveClamp(false);
      currentState = false;
      commandState = true;
    }
    else if (currentState == false){
      this.climber.setActiveClamp(true);
      currentState = true;
      commandState = true;
    }
    System.out.println("climb pivot initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (!robotContainer.getButtonB14() && commandState){
      System.out.println("climb pivot is finished");
      commandState = false;
      return true;
    }*/
    return false;
  }
}
