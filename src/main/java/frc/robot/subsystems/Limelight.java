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

public class Limelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Limelight() {
    //Initialize and Limelight parmeters
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); // 0 for force off, 3 for force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); // 0 for vision processor, 1 for driver camera
  }

  //Periodic call for limelight data
    public CommandBase getLimelight() {
        return runOnce(
        () -> {
            //Get base info and push to smartdashboard
            NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry tx = table.getEntry("tx");
            NetworkTableEntry ty = table.getEntry("ty");
            NetworkTableEntry ta = table.getEntry("ta");
            NetworkTableEntry ledmode = table.getEntry("ledMode");
            double x = tx.getDouble(0.0);
            double y = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);
            double led = ledmode.getDouble(0.0);

            //post to smart dashboard periodically
            SmartDashboard.putNumber("LimelightX", x);
            SmartDashboard.putNumber("LimelightY", y);
            SmartDashboard.putNumber("LimelightArea", area);
            SmartDashboard.putNumber("LED Mode", led);
        });
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
          /* one-time action goes here */
        });
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
