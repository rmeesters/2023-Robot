package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class BalanceRobot {
    private Swerve s_Swerve;
    final private double TOLERANCE_VALUE = 2.0; //the tolerance val is 2 degrees
    //TrajectoryConfig config;
    public BalanceRobot(){
        s_Swerve  = new Swerve();
        
    //    config =
    //         new TrajectoryConfig(
    //                 Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //             .setKinematics(Constants.Swerve.swerveKinematics);

    }


    public void balanceSwerve(){

        while(isRobotBalanced()==false){

            while(s_Swerve.gyro.getRoll()> TOLERANCE_VALUE){
           
                s_Swerve.drive(
                    new Translation2d(0.1, 0.0).times(Constants.Swerve.maxSpeed), 
                    0.0 * Constants.Swerve.maxAngularVelocity, 
                 false, 
                 true);
            
            //  GO FORWARDS
            }

            while(s_Swerve.gyro.getRoll()< TOLERANCE_VALUE){

                s_Swerve.drive(
                    new Translation2d(-0.1, 0.0).times(Constants.Swerve.maxSpeed), 
                    0.0 * Constants.Swerve.maxAngularVelocity, 
                 false, 
                 true);
                //GO BACKWARDS
            }


        }
    }
    
    public boolean isRobotBalanced(){

        if(s_Swerve.gyro.getRoll()>TOLERANCE_VALUE ||s_Swerve.gyro.getRoll()<TOLERANCE_VALUE)return false;
        else return true;
    }
    

}
