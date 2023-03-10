package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static final Joystick driver = new Joystick(0);
    // private final PS4Controller PS4 = new PS4Controller(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton lowPos = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton medPos = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton highPos = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton enableVac = new JoystickButton(driver, PS4Controller.Button.kCircle.value);  
    private final JoystickButton armHome = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    private final JoystickButton groundPickUp = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton humanPickUp = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton manualIncreaseArm = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton manualDecreasrArm = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton balanceRobot = new JoystickButton(driver, PS4Controller.Button.kPS.value);

    // private final JoystickButton limeLightModeBlink = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
 
    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final ArmSubsystem s_ArmSubsystem = new ArmSubsystem(); // Add Arm Subsystem
    //private final Limelight s_limelight = new Limelight();
    // private final LimelightsubSystem s_limelightsub = new LimelightsubSystem();
    // private final Limelight s_Limelight = new Limelight();

    /* Sendable Chooser and Autonomus Commands - need to work on this */
    private static SendableChooser<Command> autoChooser;
    private final Command m_autoLeft = new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(98, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),

        // back up across the line 
        new AutoDrive(List.of(
            (new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(-2, 0, new Rotation2d(0))),
            (new Pose2d(-4, 0, Rotation2d.fromDegrees(180+23)))
            
            ),true)
            
    );
    
    private final Command m_autoCenter =  new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(98, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        
        // back up and balance
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),
        (new Pose2d(-3.0, 0, new Rotation2d(0)))),true),
        new BalanceRobotCommand()
    );

    private final Command m_autoRight= new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(98, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),(new Pose2d(-4, 0, new Rotation2d(0)))),true)
        

            //back up  blue line 140 inches.
        // new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),
        // //if works try -4
        // //practice math: -3    
        // (new Pose2d(-1, 0, Rotation2d.fromDegrees(33.0))),
        // (new Pose2d(-1.5, 0, Rotation2d.fromDegrees(200)))),
        // true)
        //new AdjustArm2(45.0, 16.5,ArmMoveType.pickUp),
        //new AutoDrive(List.of((new Pose2d(-2.5, 0, new Rotation2d(Math.PI))),
        //    (new Pose2d(-3, -0.153, new Rotation2d(Math.PI)))),true), -- swerve to driver's right 
        //new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot)
        
        //new AutoDrive((new Pose2d(-3, -0.152, new Rotation2d(Math.PI)))),true))
        
    );
    private final Command m_testauto =  new SequentialCommandGroup(
       // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        //new AdjustArm2(98, 30.5,ArmMoveType.extendToPlace),
        //new WaitCommand(0.5),
        //new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),(new Pose2d(-1, 0, Rotation2d.fromDegrees(115)))),true)
        //90 deg
        );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
              
        // Limelight Testing
        //s_limelightsub.getDistance();
        LimelightHelpers.setStreamMode_Standard("limelight");
        LimelightHelpers.setCameraMode_Driver("limelight");
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Inside (Human Player): ", m_autoLeft);
        autoChooser.addOption("Center: ",m_autoCenter);
        autoChooser.addOption("Outside (Wall): ", m_autoRight);
        autoChooser.addOption("test: ", m_testauto);

        SmartDashboard.putData("Auto mode", autoChooser);   
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // Vacume
        enableVac.onTrue(new InstantCommand(()-> s_ArmSubsystem.toggleClaw()));

        /* Arm */
        //arm1.onTrue(new adjustArm(70,12,true,true,true));
        armHome.onTrue(new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0.25,false,false,false));
        
        lowPos.onTrue(new SequentialCommandGroup(
        //     new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
        //     new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        // new adjustArm(47.5,17,true,false,true)

        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        // new AdjustArm2(47.5, 17,ArmMoveType.extendToPlace),
        new AdjustArm2(47, 17,ArmMoveType.extendToPlace)
        // new WaitCommand(0.5),
        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        ));

        lowPos.onFalse(new SequentialCommandGroup(
        //     new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 1.0, false, true, true),
        //     new adjustArm(Constants.ArmConstants.pivotBottomAngle + 2.0, 0, true, true, true),
        // new adjustArm(47.5,17,true,false,true)

        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        // // new AdjustArm2(47.5, 17,ArmMoveType.extendToPlace),
        // new AdjustArm2(43.5, 17,ArmMoveType.extendToPlace),

        new AdjustArm2(47, 17,ArmMoveType.dropPiece),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        // new WaitCommand(0.5),
        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        ));
        // medPos.onTrue(new adjustArm(85, 15.5, false, false, true)); - try this Sequential Command:
        medPos.onTrue(new SequentialCommandGroup(

            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
            // new AdjustArm2(85, 13.75,ArmMoveType.extendToPlace),
               new AdjustArm2(84.5, 13.75,ArmMoveType.extendToPlace)
            // new WaitCommand(0.5),
            // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)



        ));
        medPos.onFalse(new SequentialCommandGroup(

        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        // // new AdjustArm2(85, 13.75,ArmMoveType.extendToPlace),
        //    new AdjustArm2(84.5, 13.75,ArmMoveType.extendToPlace),
        // new WaitCommand(0.5),
        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        new AdjustArm2(81, 11.75,ArmMoveType.dropPiece),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)


    ));


        highPos.onTrue(new SequentialCommandGroup (
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
            new AdjustArm2(97.5, 29.75,ArmMoveType.extendToPlace)
            )); 
//old 30.5
//old 2: 30
        highPos.onFalse(new SequentialCommandGroup (
                
                new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
                new WaitCommand(0.5),
                new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
                ));    
        //lowPos.whileFalse(new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0,true,false,false));
       balanceRobot.onTrue(new BalanceRobotCommand());  // Try to see if the button calls this. 
       groundPickUp.onTrue(
        new AdjustArm2(45.0, 16.5,ArmMoveType.pickUp));
       groundPickUp.onFalse(new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot));

        humanPickUp.onTrue(
        new AdjustArm2(85, 0.25,ArmMoveType.pickUp)
        //old 84
       );


        humanPickUp.onFalse(new SequentialCommandGroup(
            //new AdjustArm2(78, 0.25,ArmMoveType.placeOnRobot),
            new InstantCommand(()-> s_ArmSubsystem.toggleClaw()), 
            new WaitCommand(1),
            
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot)
        ));



        
        manualIncreaseArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0.15)));
        manualIncreaseArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));
        manualDecreasrArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(-0.15)));
        manualDecreasrArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));

        
        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        // limeLightModeBlink.onTrue(new InstantCommand(()-> s_limelight.forceOff()));
        // limeLightModeBlink.onFalse(new InstantCommand(()-> s_limelight.forceOn()));
        //limeLightModeBlink.onTrue(new Limelight.forceBlink()); 
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return autoChooser.getSelected();
    }
}
