// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private final CANSparkMax turretMotor;

  /** Creates a new Turret. */
  public Turret() {
    this.turretMotor = new CANSparkMax(Constants.turret, MotorType.kBrushless);
    this.turretMotor.getEncoder().setPosition(0);
    this.turretMotor.getEncoder().setPositionConversionFactor(Constants.turretRatio*360);
    this.turretMotor.getPIDController().setP(Constants.kPturn);
    turretMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void center(){
    double turretAdjust = Constants.kPturnCenter * this.turretMotor.getEncoder().getPosition();
    setPower(turretAdjust);
  }

  public void setPower(double power) {
    double pos = this.turretMotor.getEncoder().getPosition();
    System.out.println("Turret Position: " + pos + "°");
    if((pos > Constants.maxTurretTurn && power > 0) || (pos < -Constants.maxTurretTurn && power < 0)) {
      turretMotor.set(0);
    } else {
      turretMotor.set(power);
    }
  }

  public boolean autoTurretAlign(double x){
    double turretAdjust = 0;
    if(x > Constants.limeLightErrorAllowed) {
      turretAdjust = Constants.kPturn*(-x) + Constants.minCommand;
    } else {
      return true;
    }
    setPower(turretAdjust);
    return false;
  }

  public void manualTurretAlign(boolean right, boolean left){
    if(right) {
      setPower(Constants.turretSpeed);
    }
    else if(left) {
      setPower(-Constants.turretSpeed);
    }
    else{
      setPower(0);
    }
    
  }
}
