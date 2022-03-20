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
    limelight.getEntry("ledMode").setNumber(3);
  }

  public void ledOff(){
    limelight.getEntry("ledMode").setNumber(1);
  }

  @Override
  public void periodic() {
    tx = limelight.getEntry("tx").getDouble(0.0);
    ty = limelight.getEntry("ty").getDouble(0.0);
    tv = (limelight.getEntry("tv").getDouble(0.0) != 0);
    ta = limelight.getEntry("ta").getDouble(0.0);
    //System.out.println("tx: " + tx + ", ty: " + ty + ", tv: " + tv);
  }
  public double setShooterSpeed(){
    return (getY()*1000);
  }

  public double[] getSpeeds(){

    // Constants used to calculate motor power for alignment
    //Double Kp = (((.00222222222222222)*area)-(.021111111111111));
    Double kP = -(Constants.drivekP);
    Double min_command = Constants.driveMinCommand;
    left_command = 0;
    right_command = 0;
    turret_command = 0;


    // Checks to see if button pressec



      // Set heading error and the steering adjust
      Double heading_error = -tx;
       //Double distance_error = estimateDistance() - getIdealDistance();
      steering_adjust = (kP)*heading_error;
      // Determine power based on the horizontal offset
      if (tx > 0.5)
      {
        steering_adjust = (kP)*heading_error + min_command;
      }
      else if (tx < -0.5)
      {
        steering_adjust = (kP)*heading_error - min_command;
      }
      /*else {
              steering_adjust = 0.0;
      }*/
      
      //left_command += (steering_adjust + distance_adjust);
      //right_command += (steering_adjust - distance_adjust);
      left_command += (-steering_adjust);
      right_command += (steering_adjust);
      double[] wheelSpeeds = new double[2];
      wheelSpeeds[0] = left_command;
      wheelSpeeds[1] = right_command;

      // Run motors if the target is seen 
      if (tv){
        return wheelSpeeds;

      }
    //hi again
    
      else
      {
        return new double[3];
      }
    }

  public double estimateDistance(){
    return (Constants.targetHeight - Constants.limelightHeight) / (Math.tan(Math.toRadians(Constants.limelightAngle) + Math.toRadians(ty)));
  }

}