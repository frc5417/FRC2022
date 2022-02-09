// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class AutoClimbExtend extends CommandBase {
  private final Climber climber;
  private boolean isEnabled = false;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  /** Creates a new AutoClimb. */
  public AutoClimbExtend(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isEnabled = false;
    this.leftEncoder = this.climber.getLeftMotorEncoder();
    this.rightEncoder = this.climber.getRightMotorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.isEnabled) {
      this.isEnabled = true;
      this.climber.extendUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.leftEncoder.getPosition() >= Constants.climberExtendPos && this.rightEncoder.getPosition() >= Constants.climberExtendPos) {
      return true;
    }
    return false;
  }

}
