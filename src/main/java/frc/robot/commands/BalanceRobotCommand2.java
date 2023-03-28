// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.*;


public class BalanceRobotCommand2 extends CommandBase {
  /** Creates a new BalanceRobotCommand. */
    private Swerve s_Swerve;
    private double P;
    private double KP = 0.08;
   
    private double translationVal;
    final private double TOLERANCE_VALUE = 4.0;
    boolean robotBalanced; //the tolerance val is 2 degrees
  public BalanceRobotCommand2() {
    s_Swerve  =  RobotContainer.s_Swerve;
    addRequirements(s_Swerve);
  }

  public void balanceSwerve(){
    P = s_Swerve.gyro.getPitch()*KP;
    SmartDashboard.putBoolean("Is robot balanced", robotBalanced);
    SmartDashboard.putNumber("Roll Val", s_Swerve.gyro.getPitch());
    
    

        if(s_Swerve.gyro.getPitch()> TOLERANCE_VALUE){
          robotBalanced=false;
          
          s_Swerve.drive(
            new Translation2d(0.125, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
          //  GO FORWARDS
        }
        else if(s_Swerve.gyro.getPitch()< -(TOLERANCE_VALUE)){
          robotBalanced=false;
          s_Swerve.drive(
            new Translation2d(-0.125, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
        }
        else {
          
          robotBalanced=true;
          s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
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
