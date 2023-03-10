package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustArm2 extends SequentialCommandGroup {

	private ArmSubsystem s_ArmSubsystem;

	/** Creates a new adjustArm. */
	public AdjustArm2 (double pivotAngle, double rackPosition, ArmMoveType moveType) {
		
		switch (moveType) {
		case setPosition:
		
			addCommands(new movePivot(pivotAngle),new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)),new moveRack(rackPosition),
			new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(true)));
			break;

		case extendToPlace:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),
			 new movePivot(pivotAngle - 5.0), new moveRack(rackPosition - 2.0),new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableVac(false)),
			 new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(false)));
			break;

		case returnHome:
			addCommands(new moveRack(rackPosition),
			new InstantCommand(() -> RobotContainer.s_ArmSubsystem.enableClaw(true)),
			new movePivot(pivotAngle+4.0));
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