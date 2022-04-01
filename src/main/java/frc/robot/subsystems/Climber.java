// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Climber extends SubsystemBase {
  private CANSparkMax climbL1;
  private CANSparkMax climbL2;
  private CANSparkMax climbR1;
  private CANSparkMax climbR2;

  private Solenoid bottomClimberL;
  private Solenoid bottomClimberR;
  private Solenoid topClimberL;
  private Solenoid topClimberR;

  private DigitalInput limitSwitchR;
  private DigitalInput limitSwitchL;

  private boolean activeFlag;
  private boolean passiveFlag;
  
  public Climber() {  
   climbL1 = new CANSparkMax(Constants.climbLeft1, MotorType.kBrushless);
    climbL2 = new CANSparkMax(Constants.climbLeft2, MotorType.kBrushless);
    climbR1 = new CANSparkMax(Constants.climbRight1, MotorType.kBrushless);
    climbR2 = new CANSparkMax(Constants.climbRight2, MotorType.kBrushless);

    climbL1.setIdleMode(IdleMode.kBrake);
    climbL2.setIdleMode(IdleMode.kBrake);
    climbR1.setIdleMode(IdleMode.kBrake);
    climbR2.setIdleMode(IdleMode.kBrake);

    bottomClimberL = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbBottomLSolenoid);
    bottomClimberR = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbBottomRSolenoid);
    bottomClimberL.set(false);
    bottomClimberR.set(false);

    limitSwitchR = new DigitalInput(1);
    limitSwitchL = new DigitalInput(0);

    topClimberL = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbTopLSolenoid);
    topClimberR = new Solenoid(PneumaticsModuleType.REVPH, Constants.climbTopRSolenoid);
    topClimberL.set(false);
    topClimberR.set(false);
    passiveFlag = false;
    activeFlag = false;
    
    climbL1.setInverted(true);
    climbL2.setInverted(true);
    climbR1.setInverted(true);
    climbR2.setInverted(true);

    climbL1.getPIDController().setP(Constants.climberkP);
    climbR1.getPIDController().setP(Constants.climberkP);
    climbL2.getPIDController().setP(Constants.climberkP);
    climbR2.getPIDController().setP(Constants.climberkP);

    resetClimberPos();
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

  public void setPower(double leftPower, double rightPower) {
    //leftPower = -leftPower;

    if(Math.abs(leftPower) > .3 /*&& (leftPower < 0 || climbL1.getEncoder().getPosition() > -68)*/) {
      this.climbL1.set(-leftPower*.5);
      this.climbL2.set(-leftPower*.5);
      if(limitSwitchL.get() == false){
        if(-leftPower < 0){
          this.climbL1.set(0);
          this.climbL2.set(0);
        }
      }
      
    } else { 
      this.climbL1.set(0);
      this.climbL2.set(0);
    }
    
    if(Math.abs(rightPower) > .3 /*&& (rightPower < 0 || climbR1.getEncoder().getPosition() > -68)*/) {
      this.climbR1.set(rightPower*.5);
      this.climbR2.set(rightPower*.5); 
      if(limitSwitchR.get() == false){
        if(rightPower < 0){
          this.climbR1.set(0);
          this.climbR2.set(0);
        }
      }   
    } else {
      this.climbR1.set(0);
      this.climbR2.set(0);
    }
  }

  public void resetClimberPos(){
    climbL1.getEncoder().setPosition(0);
    climbR1.getEncoder().setPosition(0);
    climbL2.getEncoder().setPosition(0);
    climbR2.getEncoder().setPosition(0);
  }

  public void setPassiveClamp(boolean isPressed){
    if(!passiveFlag && isPressed){
      topClimberL.set(!topClimberL.get());
      topClimberR.set(!topClimberR.get());
      System.out.println("passiveclamp");
    }
    passiveFlag = isPressed;
  }

  public void setActiveClamp(boolean isPressed){
    if(!activeFlag && isPressed){
      bottomClimberL.set(!bottomClimberL.get());
      bottomClimberR.set(!bottomClimberR.get());
      System.out.println("active clamp");
    }
    activeFlag = isPressed;
  }

  public void setPosition(double position){
    climbL1.getPIDController().setReference(position, ControlType.kPosition);
    climbR1.getPIDController().setReference(position, ControlType.kPosition);
    climbL2.getPIDController().setReference(position, ControlType.kPosition);
    climbR2.getPIDController().setReference(position, ControlType.kPosition);
  }

  public RelativeEncoder getLeftMotor1Encoder(){
    return climbL1.getEncoder();
  }
  public RelativeEncoder getRightMotor1Encoder(){
    return climbR1.getEncoder();
  }
  public RelativeEncoder getLeftMotor2Encoder(){
    return climbL2.getEncoder();
  }
  public RelativeEncoder getRightMotor2Encoder(){
    return climbR2.getEncoder();
  }

}
