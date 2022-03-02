// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax climbL1;
  private CANSparkMax climbL2;
  private CANSparkMax climbR1;
  private CANSparkMax climbR2;

  
  private Solenoid bottomClimberR;
  private Solenoid bottomClimberL;
  private Solenoid topClimberR;
  private Solenoid topClimberL;

  public Climber() {  
    climbL1 = new CANSparkMax(Constants.climbLeft1, MotorType.kBrushless);
    climbL2 = new CANSparkMax(Constants.climbLeft2, MotorType.kBrushless);
    climbR1 = new CANSparkMax(Constants.climbRight1, MotorType.kBrushless);
    climbR2 = new CANSparkMax(Constants.climbRight2, MotorType.kBrushless);
    bottomClimberR = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbR1Solenoid);
    bottomClimberL = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbL1Solenoid);
    topClimberR = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbR2Solenoid);
    topClimberL = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbR1Solenoid);

    climbL2.follow(climbL1);
    climbR2.follow(climbR1);

    climbL1.setInverted(true);
    climbL2.setInverted(true);

    climbL1.getPIDController().setP(Constants.climberkP);
    climbR1.getPIDController().setP(Constants.climberkP);

    climbL1.getEncoder().setPosition(0);
    climbR1.getEncoder().setPosition(0);
    climbL2.getEncoder().setPosition(0);
    climbR2.getEncoder().setPosition(0);
  }

  public void extendUp(){
    int extendPos = Constants.climberExtendPos;
    climbL1.getPIDController().setReference(extendPos, ControlType.kPosition);
    climbR1.getPIDController().setReference(extendPos, ControlType.kPosition);
  }

  public void retract(){
    int retractPos = Constants.climberRetractPos;
    climbL1.getPIDController().setReference(retractPos, ControlType.kPosition);
    climbR1.getPIDController().setReference(retractPos, ControlType.kPosition);
  }

  public void stopClimb() {
    this.climbL1.set(0);
    this.climbL2.set(0);
    this.climbR1.set(0);
    this.climbR2.set(0);
  }

  public void setPassiveClamp(boolean deploy){
    bottomClimberR.set(deploy);
    bottomClimberL.set(deploy);
  }

  public void setActiveClamp(boolean deploy){
    topClimberR.set(deploy);
    topClimberL.set(deploy);
  }

  public void setPosition(double position){
    climbL1.getPIDController().setReference(position, ControlType.kPosition);
    climbR1.getPIDController().setReference(position, ControlType.kPosition);
  }

  public RelativeEncoder getLeftMotorEncoder(){
    return climbL1.getEncoder();
  }
  public RelativeEncoder getRightMotorEncoder(){
    return climbR1.getEncoder();
  }

/*
  public Solenoid getAnchorSolenoid() {
    return this.passiveSol;
  }

  public Solenoid getActiveSolenoid() {
    return this.activeSol;
  }
  */

}
