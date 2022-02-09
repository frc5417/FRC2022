// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class AutoClimb extends CommandBase {
  private final Climber climber;
  private boolean isEnabled = false;
  private int howLongPushedDownFor = 0;

  /** Creates a new AutoClimb. */
  public AutoClimb(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.isEnabled) {
      this.isEnabled = true;
      this.climber.extendUp();
      return;
    }

    if(this.climber.leftLimitSwitch.get() && this.climber.rightLimitSwitch.get()) {
      this.howLongPushedDownFor++;
    } else {
      this.howLongPushedDownFor = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.howLongPushedDownFor > (Constants.howLongClimbHasToBePushedDown / 20)) {
      this.howLongPushedDownFor = 0;
      this.isEnabled = false;
      return true;
    }
    return false;
  }
}
