// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 
public class Intake extends SubsystemBase {

  private final CANSparkMax rollerBar;
  private final CANSparkMax intestinePusher;
  private final CANSparkMax intestineKicker;
  private final Solenoid intakeSolenoidR;
  private final Solenoid intakeSolenoidL;

  /** Creates a new Intake. */
  public Intake() {
    this.rollerBar = new CANSparkMax(Constants.intake, MotorType.kBrushed);
    this.intestinePusher = new CANSparkMax(Constants.intestineBottom, MotorType.kBrushless);
    this.intestineKicker = new CANSparkMax(Constants.intestineKicker, MotorType.kBrushless);
    this.intakeSolenoidR = new Solenoid(PneumaticsModuleType.REVPH, Constants.intakeRSolenoid);
    this.intakeSolenoidL = new Solenoid(PneumaticsModuleType.REVPH, Constants.intakeLSolenoid);

    this.rollerBar.setIdleMode(IdleMode.kCoast);
    this.intestinePusher.setIdleMode(IdleMode.kCoast);
    this.intestineKicker.setIdleMode(IdleMode.kBrake);

    retractPistons();
  }

  // State controls the intake direction and power
  // 0 --> Off; 1 --> Forward; 2 --> Backward
  public void runIntake(int state){
    if(state == 0){
      this.rollerBar.set(0);
    }
    else if(state == 1){
      this.rollerBar.set(Constants.intakeSpeed);
    }
    else if(state == 2){
      this.rollerBar.set(-Constants.intakeSpeed);
    }
  }

  public void runIntestine(int state){
    if(state == 0){
      this.intestinePusher.set(0);
      this.intestineKicker.set(0);
    }
    else if(state == 1){
      this.intestinePusher.set(Constants.intakeSpeed/2);
      this.intestineKicker.set(-Constants.intakeSpeed);
    }
    else if(state == 2){
      this.intestinePusher.set(-Constants.intakeSpeed);
      this.intestineKicker.set(Constants.intakeSpeed);
    }
  }

  public void runIntakeSystem(int state){
    if(state == 0){
      this.rollerBar.set(0);
      this.intestinePusher.set(0);
    }
    else if(state == 1){
      this.rollerBar.set(Constants.intakeSpeed);
      this.intestinePusher.set(Constants.intakeSpeed);
    }
    else if(state == 2){
      this.rollerBar.set(-Constants.intakeSpeed);
      this.intestinePusher.set(-Constants.intakeSpeed);
    }
  }

  public void deployPistons(){
    this.intakeSolenoidR.set(true);
    this.intakeSolenoidL.set(true);
  }

  public void retractPistons(){
    this.intakeSolenoidR.set(false);
    this.intakeSolenoidL.set(false);
  }
}
