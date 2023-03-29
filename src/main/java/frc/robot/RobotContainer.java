package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
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
    private final JoystickButton findTarget = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);
    private final JoystickButton groundPickUp = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton humanPickUp = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton manualIncreaseArm = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton manualDecreasrArm = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton limelightTest = new JoystickButton(driver, PS4Controller.Button.kPS.value);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
 
    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final ArmSubsystem s_ArmSubsystem = new ArmSubsystem(); // Add Arm Subsystem


    /* Sendable Chooser and Autonomus Commands - need to work on this */
    private static SendableChooser<Command> autoChooser;
    private final Command m_autoLeft = new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
        new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),

        // back up across the line 
        new AutoDrive(List.of(
            (new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(-2, 0, new Rotation2d(0))),
            (new Pose2d(-4, 0, Rotation2d.fromDegrees(-180-23)))
            ),true)

            //old: 180+23
    );

    private final Command m_autoCenter =  new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
        new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        
        // back up and balance
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(-2.75, 0, new Rotation2d(0)))),true),
        new BalanceRobotCommand()
    );

    private final Command m_autoCenterTest = new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
        new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        new AutoDrive(List.of((new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(-3.6, 0, new Rotation2d(0)))
            //(new Pose2d(-3.5,0, new Rotation2d(0)))
            ),true),

        new WaitCommand(1),
        new AutoDrive(List.of((new Pose2d(-3.6, 0, new Rotation2d(0))),
            (new Pose2d(-4.6, 0, new Rotation2d(0)))
            //(new Pose2d(-3.5,0, new Rotation2d(0)))
            ),true),

            new AutoDrive(List.of((new Pose2d(-4.6, 0, new Rotation2d(0))),
            (new Pose2d(-2.7, 0, new Rotation2d(0)))
            ),false),
            //old 2.7
            new BalanceRobotCommand()


    );

    private final Command m_autoRight= new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
        new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        
        // new AdjustArm2(98, 29.75,ArmMoveType.extendToPlace),
        // new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
        // new WaitCommand(0.5),
        // new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome),
        new AutoDrive(List.of(
            (new Pose2d(0, 0, new Rotation2d(0))),
            (new Pose2d(-2, 0.3, new Rotation2d(0))),
            (new Pose2d(-4, 0.3, Rotation2d.fromDegrees(180+23))),
            (new Pose2d(-4.7,0.3, Rotation2d.fromDegrees(180+23)))
            //old -4.3
            ),true),

        //changes
        new AdjustArm2(45.0, 16.5,ArmMoveType.pickUp),
        new WaitCommand(0.5),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot)
    );

        
        
        // Negative y value will move Right relative to driver - e.g. (new Pose2d(-3, -0.153, new Rotation2d(Math.PI)))),true) should move .153m to the right
    

    private final Command m_testauto =  new SequentialCommandGroup(
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
        new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
        new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace),
        new AdjustArm2(94, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.1),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
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
              
        // Limelight Configuration (for driver cam)
        LimelightHelpers.setStreamMode_Standard("limelight");
        LimelightHelpers.setCameraMode_Driver("limelight");
        LimelightHelpers.setLEDMode_ForceOff("limelight");
        
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Inside (Human Player): ", m_autoLeft);
        autoChooser.addOption("Center: ",m_autoCenter);
        autoChooser.addOption("Outside (Wall): ", m_autoRight);
        autoChooser.addOption("test: ", m_testauto);
        autoChooser.addOption("center cross line", m_autoCenterTest);

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
        // Vacuum
        enableVac.onTrue(new InstantCommand(()-> s_ArmSubsystem.toggleClaw()));

        findTarget.onTrue(new SequentialCommandGroup(



        ));

        /* Arm */
        armHome.onTrue(new adjustArm(Constants.ArmConstants.pivotBottomAngle+2,0.25,false,false,false));
        
        lowPos.onTrue(new SequentialCommandGroup(
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
            new AdjustArm2(47, 17,ArmMoveType.extendToPlace)
        ));

        lowPos.onFalse(new SequentialCommandGroup(
            new AdjustArm2(47, 17,ArmMoveType.dropPiece),
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        ));
        
        medPos.onTrue(new SequentialCommandGroup(
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
            new AdjustArmParallel(86.5, 13.75,ArmMoveType.extendToPlace)
            
            //old: 85.5

        ));
        
        medPos.onFalse(new SequentialCommandGroup(
            new AdjustArm2(79, 11.75,ArmMoveType.dropPiece),
            //old 81 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        ));

        highPos.onTrue(new SequentialCommandGroup (
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4.0, 0.25,ArmMoveType.setPosition),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+42.75,0,ArmMoveType.setPosition),
            new AdjustArmParallel(98.5, 29.75,ArmMoveType.extendToPlace)
        )); 

        highPos.onFalse(new SequentialCommandGroup (
            new AdjustArm2(92, 28.5,ArmMoveType.dropPiece),
            //old 94 angle
            new WaitCommand(0.5),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle, 0.25,ArmMoveType.returnHome)
        ));    

       //balanceRobot.whileTrue(new BalanceRobotCommand());  // Try to see if the button calls this. 
       limelightTest.whileTrue(new LimelightTarget()); 
       groundPickUp.onTrue(
           new AdjustArm2(45.0, 16.5,ArmMoveType.pickUp));
       
        groundPickUp.onFalse(
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot));

        humanPickUp.onTrue(
            new AdjustArm2(86, 0.25,ArmMoveType.pickUp)
        );

        humanPickUp.onFalse(new SequentialCommandGroup(
            new InstantCommand(()-> s_ArmSubsystem.toggleClaw()), 
            new WaitCommand(1),
            new AdjustArm2(Constants.ArmConstants.pivotBottomAngle+4,0.25,ArmMoveType.placeOnRobot)
        ));
        
        manualIncreaseArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0.15)));
        manualIncreaseArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));
        manualDecreasrArm.whileTrue(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(-0.15)));
        manualDecreasrArm.onFalse(new InstantCommand(()-> s_ArmSubsystem.manualLiftArm(0)));

        // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
