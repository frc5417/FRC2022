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
  private double position = -1.0;

  /** Creates a new AutoClimb. */
  public AutoClimbExtend(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  public AutoClimbExtend(Climber climber, double position) {
    this.climber = climber;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isEnabled = false;
    this.leftEncoder = this.climber.getLeftMotor1Encoder();
    this.rightEncoder = this.climber.getRightMotor1Encoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.isEnabled) {
      this.isEnabled = true;
      if(position == -1.0) {
        this.climber.extendUp();
      } else {
        this.climber.setPosition(position);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.position == -1.0) {
      if(this.leftEncoder.getPosition() >= Constants.climberExtendPos && this.rightEncoder.getPosition() >= Constants.climberExtendPos) {
        return true;
      }
    } else {
      if(this.leftEncoder.getPosition() >= this.position && this.rightEncoder.getPosition() >= this.position) {
        return true;
      }
    }

    return false;
  }

  public void setPosition(double position) {
    this.position = position;
  }

}
