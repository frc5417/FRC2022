// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
public class Intake extends SubsystemBase {


  private CANSparkMax rollerBar;
  private CANSparkMax internalPusher;
  private Solenoid intakeSolenoid;

  /** Creates a new Intake. */
  public Intake() {
    CANSparkMax rollerBar = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax internalPusher = new CANSparkMax(5, MotorType.kBrushless);


    rollerBar.setIdleMode(IdleMode.kCoast);
    internalPusher.setIdleMode(IdleMode.kCoast);
    


  }

  public void runIntake(){
    rollerBar.set(Constants.intakeSpeed);

  }

  public void deployPistons(){
    intakeSolenoid.set(true);
  }

  public void retractPistons(){
    intakeSolenoid.set(false);
  }

  public void intestine(){
    internalPusher.set(Constants.pusherSpeed);
  }
}
