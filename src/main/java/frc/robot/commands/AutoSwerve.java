package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private double translationSup;
    private double strafeSup;
    private double rotationSup;
    private boolean robotCentricSup;
    
    public AutoSwerve(Swerve s_Swerve, double translationSup, double strafeSup, double rotationSup, boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() { 
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationSup, strafeSup).times(Constants.Swerve.maxSpeed), 
            rotationSup * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );
    }
}