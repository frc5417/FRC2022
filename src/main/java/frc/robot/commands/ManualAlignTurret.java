// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class ManualAlignTurret extends CommandBase {
  /** Creates a new ManualAlignTurret. */
  private final Turret turret;
  private final RobotContainer robotContainer;

  public ManualAlignTurret(RobotContainer robotContainer, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.robotContainer = robotContainer;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.robotContainer.getDpadUp()){
      this.turret.center();
    }
    this.turret.manualTurretAlign(robotContainer.getDpadRight(), robotContainer.getDpadLeft());
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
