package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustArm2 extends SequentialCommandGroup {

    private ArmSubsystem s_ArmSubsystem;    
	/** Creates a new adjustArm. */
	public AdjustArm2(double pivotAngle, double rackPosition, ArmMoveType moveType) {
		switch (moveType) {
		case GrabPiece:
			addCommands(new moveRack(rackPosition), new movePivot(pivotAngle),new InstantCommand(() -> s_ArmSubsystem.enableVac(true)));
			break;
		case GrabPiecePivotFirst:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),new InstantCommand(() -> s_ArmSubsystem.enableVac(true)));
			break;
		case DropPiece:
			addCommands(new moveRack(rackPosition), new movePivot(pivotAngle),new InstantCommand(() -> s_ArmSubsystem.enableVac(false)));
			break;
		case DropPiecePivotFirst:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),new InstantCommand(() -> s_ArmSubsystem.enableVac(false)));
			break;
		case PutInClaw:
			addCommands(new movePivot(pivotAngle), new moveRack(rackPosition),new InstantCommand(() -> s_ArmSubsystem.enableClaw(true)),
					new InstantCommand(() -> s_ArmSubsystem.enableVac(false)));
			break;
		case TakeFromClaw:
			addCommands(new InstantCommand(() -> s_ArmSubsystem.enableVac(true)), 
					new InstantCommand(() -> s_ArmSubsystem.enableClaw(false)),new movePivot(pivotAngle), new moveRack(rackPosition));
			break;
		}
	}
}