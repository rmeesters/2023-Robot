// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers.*;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.*;
import frc.robot.LimelightHelpers;


public class LimelightTarget extends CommandBase {
  /** Creates a new LimelightTarget. */
  private Swerve s_Swerve;
  double distancexAway;
  double distanceyAway;

  double cameraOffset=5;
  public LimelightTarget() {
    s_Swerve  =  RobotContainer.s_Swerve;
    addRequirements(s_Swerve);
   
  }

  public void drivetoTarget(){
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
    var rrResults = llresults.targetingResults.targets_Retro[0];
    
    // LimelightTarget_Retro target = new LimelightTarget_Retro();
     Pose2d targetpos =  rrResults.getRobotPose_TargetSpace2D();
     //LimelightResults r = new LimelightResults();

    distancexAway = LimelightHelpers.getTX("limelight"); //target.tx;
    distanceyAway = targetpos.getY();

    //SmartDashboard.putNumber("distance x away from target", r.targetingResults.getBotPose2d().getX());
    SmartDashboard.putNumber("TX Angle:", distancexAway);
    SmartDashboard.putNumber("distance y away from target", distanceyAway);


    //can change later.

  //   s_Swerve.drive(
  //     new Translation2d(0, distancexAway-cameraOffset).times(Constants.Swerve.maxSpeed), //0.125
  //     0 * Constants.Swerve.maxAngularVelocity, 
  //     false, 
  //     true
  //     //old +0.125
  // );

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetoTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
