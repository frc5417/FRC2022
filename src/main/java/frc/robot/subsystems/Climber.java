// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax climbL1;
  private CANSparkMax climbL2;
  private CANSparkMax climbR1;
  private CANSparkMax climbR2;

  private Solenoid bottomClimber;
  private Solenoid topClimber;

  private boolean activeFlag;
  private boolean passiveFlag;
  
  public Climber() {  
/*    climbL1 = new CANSparkMax(Constants.climbLeft1, MotorType.kBrushless);
    climbL2 = new CANSparkMax(Constants.climbLeft2, MotorType.kBrushless);
    climbR1 = new CANSparkMax(Constants.climbRight1, MotorType.kBrushless);
    climbR2 = new CANSparkMax(Constants.climbRight2, MotorType.kBrushless); */
    // climbL1 = new CANSparkMax(13, MotorType.kBrushless);
    // climbL2 = new CANSparkMax(2, MotorType.kBrushless);
    // climbR1 = new CANSparkMax(5, MotorType.kBrushless);
    // climbR2 = new CANSparkMax(4, MotorType.kBrushless);
    // bottomClimber = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.climbBottomSolenoid);
    // bottomClimber.set(false);
    topClimber = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.climbTopSolenoid);

    passiveFlag = false;
    activeFlag = false;
    
    // climbL1.setInverted(false);
    // climbL2.setInverted(false);
    // climbR1.setInverted(false);
    // climbR2.setInverted(false);


    // climbL1.getPIDController().setP(Constants.climberkP);
    // climbR1.getPIDController().setP(Constants.climberkP);
    // climbL2.getPIDController().setP(Constants.climberkP);
    // climbR2.getPIDController().setP(Constants.climberkP);

    resetClimberPos();
  }

  public void extendUp(){
    // int extendPos = Constants.climberExtendPos;
    // climbL1.getPIDController().setReference(extendPos, ControlType.kPosition);
    // climbR1.getPIDController().setReference(extendPos, ControlType.kPosition);
  }

  public void retract(){
    int retractPos = Constants.climberRetractPos;
    // climbL1.getPIDController().setReference(retractPos, ControlType.kPosition);
    // climbR1.getPIDController().setReference(retractPos, ControlType.kPosition);
  }

  public void stopClimb() {
    // this.climbL1.set(0);
    // this.climbL2.set(0);
    // this.climbR1.set(0);
    // this.climbR2.set(0);
  }

  public void setPower(double leftPower, double rightPower) {
    /*
    if(climbL1.getEncoder().getPosition() > 0 || climbL2.getEncoder().getPosition() > 0){
      this.climbL1.getPIDController().setReference(0, ControlType.kPosition);
      this.climbL2.getPIDController().setReference(0, ControlType.kPosition);
    }*/
    /*
    if(Math.abs(leftPower) > .1){
      this.climbL1.set(leftPower*.25);
      this.climbL2.set(leftPower*.25);
    }
    else{
      this.climbL1.set(0);
      this.climbL2.set(0);
    }*/

    /*if(climbR1.getEncoder().getPosition() < 0 || climbR2.getEncoder().getPosition() < 0){
      this.climbR1.getPIDController().setReference(0, ControlType.kPosition);
      this.climbR2.getPIDController().setReference(0, ControlType.kPosition);
    }*/
    /*
    if(Math.abs(rightPower) > .1){
      this.climbR1.set(rightPower*.25);
      this.climbR2.set(rightPower*.25);
    }
    else{
      this.climbR1.set(0);
      this.climbR2.set(0);
    }

    */
  }

  public void resetClimberPos(){
    return;
    // climbL1.getEncoder().setPosition(0);
    // climbR1.getEncoder().setPosition(0);
    // climbL2.getEncoder().setPosition(0);
    // climbR2.getEncoder().setPosition(0);
  }


  public void setPassiveClamp(boolean isPressed){
    if(!passiveFlag && isPressed){
      topClimber.set(!topClimber.get());
    }
    passiveFlag = isPressed;
  }

  public void setActiveClamp(boolean isPressed){
    if(!activeFlag && isPressed){
      // bottomClimber.set(!bottomClimber.get());
    }
    activeFlag = isPressed;
  }

  public void setPosition(double position){
    // climbL1.getPIDController().setReference(position, ControlType.kPosition);
    // climbR1.getPIDController().setReference(position, ControlType.kPosition);
    // climbL2.getPIDController().setReference(position, ControlType.kPosition);
    // climbR2.getPIDController().setReference(position, ControlType.kPosition);
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

/*
  public Solenoid getAnchorSolenoid() {
    return this.passiveSol;
  }

  public Solenoid getActiveSolenoid() {
    return this.activeSol;
  }
  */

}
