// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private final CANSparkMax turretMotor;

  /** Creates a new Turret. */
  public Turret() {
    this.turretMotor = new CANSparkMax(Constants.turret, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    turretMotor.setIdleMode(IdleMode.kCoast);

    turretMotor.set(power);
  }

  public boolean autoTurretAlign(double x){
    double turretAdjust = 0;
    if(x > Constants.limeLightErrorAllowed){
      turretAdjust = Constants.kPturn*(-x) + Constants.minCommand;
    }else{
      return true;
    }
    setPower(turretAdjust);
    return false;
  }

  public void manualTurretAlign(boolean right, boolean left){
    if(right){
      setPower(Constants.turretSpeed);
    }
    else if(left){
      setPower(-Constants.turretSpeed);
    }
    else{
      setPower(0);
    }
    
  }
}
