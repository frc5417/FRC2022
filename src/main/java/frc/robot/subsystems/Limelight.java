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
  private double left_command;
  private double right_command;
  private double turret_command;
  private double steering_adjust;
  private double distance_adjust;
  private double accumulated_heading_error = 0;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    ledOff();
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
    limelight.getEntry("ledMode").setNumber(3);
  }

  public void ledOff(){
    limelight.getEntry("ledMode").setNumber(1);//should be one, change before pushing to remote
  }

  @Override
  public void periodic() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = (limelight.getEntry("tv").getDouble(0.0) != 0);
    ta = limelight.getEntry("ta").getDouble(0.0);
    //Constants.shootsetPointVar = (int)(Math.pow(Math.E, -0.125245 * ty + 5.93303) + 2718.9); //3750; //(int)(-476.224*Math.tanh(.159 * ty + .775) + 3008.3);
    //System.out.println("tx: " + tx + ", ty: " + ty + ", tv: " + tv);
  }

  public double[] getSpeeds(){

    // Constants used to calculate motor power for alignment
    // Double Kp = (((.00222222222222222)*area)-(.021111111111111));
    Double kP = -(Constants.drivekP);
    Double kI = Constants.drivekI;
    Double kPDistance = -.0045;
    Double min_command = Constants.driveMinCommand;
    left_command = 0;
    right_command = 0;
    turret_command = 0;


    // Checks to see if button pressec

    // Set heading error and the steering adjust
    Double heading_error = -tx;

    Double distance_error = -ty;
    // Double distance_error = estimateDistance() - getIdealDistance();
    steering_adjust = (kP)*heading_error;
    distance_adjust = (kP)*distance_error;
    // Determine power based on the horizontal offset
    if (tx > 1.0) {
      steering_adjust = (kP)*heading_error + (kI)*accumulated_heading_error + min_command;
    } else if (tx < -1.0) {
      steering_adjust = (kP)*heading_error + (kI)*accumulated_heading_error - min_command;
    }
    
    if (Math.abs(accumulated_heading_error) >= 100){
      accumulated_heading_error = Math.ceil(accumulated_heading_error);
    }
    else{ 
      accumulated_heading_error += heading_error;
    }
    //distance_adjust = kPDistance * distance_error;
    distance_adjust = 0;
    // left_command += (distance_adjust + steering_adjust);
    // right_command += (steering_adjust - distance_adjust);
    left_command += (-steering_adjust);
    right_command += (steering_adjust);
    double[] wheelSpeeds = new double[2];
    wheelSpeeds[0] = left_command;
    wheelSpeeds[1] = right_command;

    // Run motors if the target is seen 
    if (tv){
      return wheelSpeeds;

    } else {
      return new double[2];
    }
  }

  public double estimateDistance(){
    return (Constants.targetHeight - Constants.limelightHeight) / (Math.tan(Math.toRadians(Constants.limelightAngle) + Math.toRadians(ty)));
  }

}