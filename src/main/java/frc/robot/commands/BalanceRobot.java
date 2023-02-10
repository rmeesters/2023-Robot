package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import com.kauailabs.navx.frc.AHRS;


public class BalanceRobot {
    private double initialAngle;
    private Swerve s_Swerve;
    private AHRS gyro;
    
    public BalanceRobot(){
        s_Swerve  = new Swerve();
        initialAngle = gyro.getRoll();

        while (initialAngle > 2.0 || initialAngle < -2.0) {
            if (initialAngle > 0.0){
                s_Swerve.drive(
                    new Translation2d(0.1, 0.0).times(Constants.Swerve.maxSpeed), 
                    0.0 * Constants.Swerve.maxAngularVelocity, 
                 false, 
                 true);
            }
            else if (initialAngle < 0.0) {
                s_Swerve.drive(
                    new Translation2d(-0.1, 0.0).times(Constants.Swerve.maxSpeed), 
                    0.0 * Constants.Swerve.maxAngularVelocity, 
                 false, 
                 true);    
            }
            else {
                s_Swerve.drive(
                    new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
                    0.0 * Constants.Swerve.maxAngularVelocity, 
                 false, 
                 true);
            }
        }
  
    }
    
    // public double angleOffset(){

        
    // }

    // read the angular velocity data from the gyro sensor.
    // Use the gyro data to calculate the angle of the robot's tilt.
    // Control the motors to correct the tilt angle and maintain balance.
    // Integrate the code with the rest of your robot's hardware, such as the motors and wheels, to create a complete self-balancing robot system.

}
