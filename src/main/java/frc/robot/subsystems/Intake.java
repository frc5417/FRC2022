// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
public class Intake extends SubsystemBase {

  private final CANSparkMax rollerBar;
  private final CANSparkMax intestinePusher;
  private final CANSparkMax intestineKicker;
  private final Solenoid intakeSolenoid;

  /** Creates a new Intake. */
  public Intake() {
    this.rollerBar = new CANSparkMax(Constants.intake, MotorType.kBrushless);
    this.intestinePusher = new CANSparkMax(Constants.intestineBottom, MotorType.kBrushless);
    this.intestineKicker = new CANSparkMax(Constants.intestineKicker, MotorType.kBrushless);
    this.intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    rollerBar.setIdleMode(IdleMode.kCoast);
    intestinePusher.setIdleMode(IdleMode.kCoast);
    intestineKicker.setIdleMode(IdleMode.kCoast);
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

  public void runIntestine(){
    intestinePusher.set(Constants.pusherSpeed);
    intestineKicker.set(-Constants.pusherSpeed);
  }
}
