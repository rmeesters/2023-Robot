// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.autos.*;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.autos.*;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class BalanceRobotCommand extends CommandBase {
  /** Creates a new BalanceRobotCommand. */

  private Swerve s_Swerve;
    private final double SPEED_BALANCING = 1;
    final private double TOLERANCE_VALUE = 2.0;
    boolean robotBalanced; //the tolerance val is 2 degrees
  public BalanceRobotCommand() {
    s_Swerve  =  RobotContainer.s_Swerve;
  }

  public void balanceSwerve(){
    SmartDashboard.putBoolean("Is robot balanced", robotBalanced);
    SmartDashboard.putNumber("Roll Val", s_Swerve.gyro.getPitch());

        if(s_Swerve.gyro.getPitch()> TOLERANCE_VALUE){
          robotBalanced=false;
          new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),(new Pose2d(-0.1, 0, new Rotation2d(0)))),true);
                
        //  GO FORWARDS
        }

        else if(s_Swerve.gyro.getPitch()< -(TOLERANCE_VALUE)){

          robotBalanced=false;
          new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),(new Pose2d(0.1, 0, new Rotation2d(0)))),false);
              
           // GO BACKWARDS

        }

        else {
            
            robotBalanced=true;

           
        }


    
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    balanceSwerve();
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
