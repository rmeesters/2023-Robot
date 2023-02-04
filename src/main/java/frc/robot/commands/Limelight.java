// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;

/** An example command that uses an example subsystem. */
public class Limelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ledmode;
  double x;
  double y;
  double area;
  double led;
  final double limelightmountAnglesDegrees= 0.0; //total angle - ty 
  final double limelightLensHeightInches=39.5;
  final double goalHeightInches= 29.5;
  final double distanceDiffToTarget = -9.25;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Limelight() {
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // 0 for force off, 3 for
    // force on
NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // 0 for vision processor,
    // 1 for driver camera
table = NetworkTableInstance.getDefault().getTable("limelight");
tx = table.getEntry("tx");
ty = table.getEntry("ty");
ta = table.getEntry("ta");
ledmode = table.getEntry("ledMode");
x = tx.getDouble(0.0);
y = ty.getDouble(0.0);
area = ta.getDouble(0.0);
led = ledmode.getDouble(0.0);

// post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
SmartDashboard.putNumber("LED Mode", led);

    
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double getTargetArea() {
    double a = ta.getDouble(0.0);
    return a;
  }

  public double getdegRotationToTarget() {
  
    double x = tx.getDouble(0.0);
    return x;
}

public double getdegVerticalToTarget() {
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


}
