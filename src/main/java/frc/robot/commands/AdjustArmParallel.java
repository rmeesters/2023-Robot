package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustArmParallel extends ParallelCommandGroup {

	private ArmSubsystem s_ArmSubsystem;

	/** Creates a new adjustArm. */
	public AdjustArmParallel (double pivotAngle, double rackPosition, ArmMoveType moveType) {
		
		switch (moveType) {
		case setPosition:
		
			addCommands(new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(true)), new movePivot(pivotAngle),new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)),new moveRack(rackPosition)
			);
			break;

		case extendToPlace:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),
			 new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(true)),
			 new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)));
			break;

		case dropPiece:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),
			new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(false)),
			new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)));
	  	    break;


		case returnHome:
			addCommands(new moveRack(rackPosition),
			new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)),
			new movePivot(pivotAngle+4.0), new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)));
			break;
		
		case pickUp:
			addCommands(new movePivot(pivotAngle), new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)), 
			new moveRack(rackPosition), new InstantCommand(() ->RobotContainer.s_ArmSubsystem.enableVac(true)));
			break;
		case placeOnRobot:
			addCommands(new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)),
				new moveRack(rackPosition), new movePivot(pivotAngle),
			 new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(false)));
			break;
		case PutInClaw:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)),
					new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(false)));
			break;
		case TakeFromClaw:
			addCommands(new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(true)), 
					new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)),new movePivot(pivotAngle), new moveRack(rackPosition));
			break;
	
		}
	}
}