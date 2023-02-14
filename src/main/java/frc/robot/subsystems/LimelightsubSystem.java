// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
//import oi.limelightvision.limelight.frc.ControlMode.*;

public class LimelightsubSystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

 
  final double limelightmountAnglesDegrees= 0.0;
  final double limelightLensHeightInches=39.5;
  final double goalHeightInches= 29.5;
  final double distanceDiffToTarget = -9.25;
  NetworkTable table;
  NetworkTableEntry ledmode;
  double led;
  

  public LimelightsubSystem() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // 0 for force off, 3 for
    // force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // 0 for vision processor,
    // 1 for driver camera
    table = NetworkTableInstance.getDefault().getTable("limelight");
    ledmode = table.getEntry("ledMode");

    led = ledmode.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", table.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("LimelightArea", table.getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumber("LED Mode", led);

  }



  /**
   * Put in code to get distance here
   *
   * @return a command
   */
  public CommandBase getDistance() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          SmartDashboard.putNumber("Distance to Goal in Inches:", this.DistanceToGoalInInches());
          System.out.println(this.DistanceToGoalInInches());
        });
  }

  public double getTargetArea() {
    NetworkTableEntry ta = table.getEntry("ta");
    double area = ta.getDouble(0.0);
    return area;
  }

  public double getdegRotationToTarget() {
    NetworkTableEntry tx = table.getEntry("tx");
   double x = tx.getDouble(0.0);
    return x;
}

public double getdegVerticalToTarget() {
  NetworkTableEntry ty = table.getEntry("ty");
  double y = ty.getDouble(0.0);
  return y;
}

public double getSkew_Rotation() {
  NetworkTableEntry ts = table.getEntry("ts");
  double s = ts.getDouble(0.0);
  return s;
}

public double getPipelineLatency() {
  NetworkTableEntry tl = table.getEntry("tl");
  double l = tl.getDouble(0.0);
  return l;
}


private void resetPilelineLatency(){
  table.getEntry("tl").setValue(0.0);

}

public double getAngletoGoalDegrees(){

  return limelightmountAnglesDegrees + this.getdegVerticalToTarget();
  

}
public double getAngletoGoalRadians(){
  return this.getAngletoGoalDegrees() * (Math.PI / 180);

}

public double DistanceToGoalInInches(){
  return (goalHeightInches- limelightLensHeightInches)/Math.tan(this.getAngletoGoalRadians());
}
//cam controls
public void forceOff(){
  SmartDashboard.putString("Limelight ON:", "False");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  // table.getEntry("ledMode").setValue(1);
}

public void forceOn(){
  SmartDashboard.putString("Limelight ON:", "True");
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
  //table.getEntry("ledMode").setValue(3);
}

public void forceBlink(){
  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
  table.getEntry("ledMode").setValue(2);
}


public double getCamMode() {
  NetworkTableEntry camMode = table.getEntry("camMode");
  double mode = camMode.getDouble(0.0);
  SmartDashboard.putNumber("Camera Mode", mode);
  return mode;
}
public void setCamMode(int camMode){

  table.getEntry("camMode").setValue(camMode);

}

public void setPipeline(Integer pipeline) {
  if(pipeline<0){
      pipeline = 0;
      throw new IllegalArgumentException("Pipeline can not be less than zero");
  }else if(pipeline>9){
      pipeline = 9;
      throw new IllegalArgumentException("Pipeline can not be greater than nine");
  }
  table.getEntry("pipeline").setValue(pipeline);
}

public double getPipeline(){
  NetworkTableEntry pipeline = table.getEntry("pipeline");
  double pipe = pipeline.getDouble(0.0);
  return pipe;
}

public Integer getPipelineInt(){
  NetworkTableEntry pipeline = table.getEntry("pipeline");
  Integer pipe = (int) pipeline.getDouble(0.0);
  return pipe;
}

public boolean getIsTargetFound() {
  NetworkTableEntry tv = table.getEntry("tv");
  double v = tv.getDouble(0);
  if (v == 0.0){
      return false;
  }else {
      return true;
  }
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
