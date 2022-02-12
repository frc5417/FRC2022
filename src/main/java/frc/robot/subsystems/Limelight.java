// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private NetworkTable limelight;
  private double tx, ty, ta;
  private boolean tv;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public boolean getV(){
    return tv;
  }

  public double getX(){
    return tx;
  }

  public double getY(){
    return ty;
  }

  public double getA(){
    return ta;
  }

  public void ledOn(){
    limelight.getEntry("ledMode").setDouble(3.0);
  }

  public void ledOff(){
    limelight.getEntry("ledMode").setDouble(1.0);
  }

  @Override
  public void periodic() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = (limelight.getEntry("tv").getDouble(0.0) != 0);
    ta = limelight.getEntry("ta").getDouble(0.0);
    //System.out.println("tx: " + tx + ", ty: " + ty + ", tv: " + tv);
  }

  public double estimateDistance(){
    return (Constants.targetHeight - Constants.limelightHeight) / (Math.tan(Math.toRadians(Constants.limelightAngle) + Math.toRadians(y)));
  }

}