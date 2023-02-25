// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class BalanceRobotCommand extends CommandBase {
  /** Creates a new BalanceRobotCommand. */
    private Swerve s_Swerve;
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
          new AutoSwerve(s_Swerve, 0.2, 0.0, 0.0, false);
          //  GO FORWARDS
        }
        else if(s_Swerve.gyro.getPitch()< -(TOLERANCE_VALUE)){
          robotBalanced=false;
          new AutoSwerve(s_Swerve, -0.2, 0.0, 0.0, false);
        }
        else {
          robotBalanced=true;
          new AutoSwerve(s_Swerve, 0.0, 0.0, 0.0, false);
          // isFinished();
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
