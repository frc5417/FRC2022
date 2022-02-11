// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.RobotContainer;

public class AutoTankDrive extends CommandBase {
  private final Drive drive;
  private boolean isEnabled = false;
  private RobotContainer robotContainer;
  private int howLongActiveFor = 0;

  public AutoTankDrive(Drive drive, RobotContainer container) {
    this.drive = drive;
    this.robotContainer = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isEnabled = false;
    this.howLongActiveFor = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.isEnabled) {
      this.drive.setPower(-.5, -.5);
      this.isEnabled = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.howLongActiveFor > 500) {
      this.drive.setPower(0, 0);
      this.howLongActiveFor = 0;
      this.isEnabled = false;
      return true;
    }
    return false;
  }
}
