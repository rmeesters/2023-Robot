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

public class Limelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry ledmode;
  double x;
  double y;
  double area;
  double led;
  final double limelightmountAnglesDegrees= 0.0;
  final double limelightLensHeightInches=39.5;
  final double goalHeightInches= 29.5;
  final double distanceDiffToTarget = -9.25;

  

  public Limelight() {
    // Initialize and Limelight parmeters
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

  // Periodic call for limelight data
  // public CommandBase getLimelight() {
  //   return {
  //         // Get base info and push to smartdashboard
  //         table = NetworkTableInstance.getDefault().getTable("limelight");
  //         tx = table.getEntry("tx");
  //         ty = table.getEntry("ty");
  //         ta = table.getEntry("ta");
  //         ledmode = table.getEntry("ledMode");
  //         x = tx.getDouble(0.0);
  //         y = ty.getDouble(0.0);
  //         area = ta.getDouble(0.0);
  //         led = ledmode.getDouble(0.0);

  //         // post to smart dashboard periodically
  //         SmartDashboard.putNumber("LimelightX", x);
  //         SmartDashboard.putNumber("LimelightY", y);
  //         SmartDashboard.putNumber("LimelightArea", area);
  //         SmartDashboard.putNumber("LED Mode", led);
  //       };
  // }

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
          System.out.println(this.DistanceToGoalInInches());
        });
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


// public void setLEDMode(LedMode ledMode) {
//   table.getEntry("ledmode").setValue(ledMode.getValue());
// }

// public LedMode getLEDMode() {
//   NetworkTableEntry ledMode = table.getEntry("ledMode");
//   double led = ledMode.getDouble(0.0);
//   LedMode mode = LedMode.getByValue(led);
//   return mode;
// }

// public CamMode getCamMode() {
//   NetworkTableEntry camMode = table.getEntry("camMode");
//   double cam = camMode.getDouble(0.0);
//   CamMode mode = CamMode.getByValue(cam);
//   return mode;
// }

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

// public void setStream(StreamType stream) {
//   table.getEntry("stream").setValue(stream.getValue());
// }

// public StreamType getStream() {
//   NetworkTableEntry stream = table.getEntry("stream");
//   double st = stream.getDouble(0.0);
//   StreamType mode = StreamType.getByValue(st);
//   return mode;
// }
// public void setSnapshot(Snapshot snapshot) {
//   table.getEntry("snapshot").setValue(snapshot.getValue());
// }

// public Snapshot getSnapshot() {
//   NetworkTableEntry snapshot = table.getEntry("snapshot");
//   double snshot = snapshot.getDouble(0.0);
//   Snapshot mode = Snapshot.getByValue(snshot );        
//   return mode;
// }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
