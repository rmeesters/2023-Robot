package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;

public class ArmSubsystem extends SubsystemBase {
    /* Arm Setup */
    private WPI_TalonFX armPivot;
    private WPI_TalonFX armRack;
    public int loopIDX = Constants.ArmConstants.loopIDX;
    public int timeoutMS = Constants.ArmConstants.timeoutMS;
    public double pivotKP = Constants.ArmConstants.armPivotKP;
    public double pivotKI = Constants.ArmConstants.armPivotKI;
    public double pivotKD = Constants.ArmConstants.armPivotKD;
    public double pivotKF = Constants.ArmConstants.armPivotKF;
    public double rackKP = Constants.ArmConstants.armRackKP;
    public double rackKI = Constants.ArmConstants.armRackKI;
    public double rackKD = Constants.ArmConstants.armRackKI;
    public double rackKF = Constants.ArmConstants.armRackKF;

    /* Solenoid Setup */
    public int airSupplyCAN = Constants.ArmConstants.airSupplyCAN;
    private PneumaticHub armPH;
    /* Arm Absolute Encoder? */

    public ArmSubsystem() {
        armPivot = new WPI_TalonFX(Constants.ArmConstants.armPivot);
        armRack = new WPI_TalonFX(Constants.ArmConstants.armRack);
        armPH = new PneumaticHub(airSupplyCAN);

        // Configure Arm Defaults
        armPivot.configFactoryDefault();
        armPivot.setSelectedSensorPosition(0);
        armPivot.setNeutralMode(NeutralMode.Brake);
        armRack.configFactoryDefault();
        armRack.setSelectedSensorPosition(0);
        armRack.setNeutralMode(NeutralMode.Brake);

        // Config feedback sensors for PID
        armPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeoutMS);
        armRack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeoutMS);

        // Config peak and nominal outputs
        armPivot.configNominalOutputForward(0, timeoutMS);
        armPivot.configNominalOutputReverse(0, timeoutMS);
        armPivot.configPeakOutputForward(0.5, timeoutMS);
        armPivot.configPeakOutputForward(0.5, timeoutMS);
        armPivot.setInverted(true);
        armPivot.setSensorPhase(true);
        armRack.configNominalOutputForward(0, timeoutMS);
        armRack.configNominalOutputReverse(0, timeoutMS);
        armRack.configPeakOutputForward(0.5, timeoutMS);
        armRack.configPeakOutputForward(0.5, timeoutMS);
        armRack.setInverted(true);
        armRack.setSensorPhase(true);

        // Config P, I, D, F values
        armPivot.config_kP(loopIDX, pivotKP, timeoutMS);
        armPivot.config_kI(loopIDX, pivotKI, timeoutMS);
        armPivot.config_kD(loopIDX, pivotKD, timeoutMS);
        armPivot.config_kF(loopIDX, pivotKF, timeoutMS);
        armRack.config_kP(loopIDX, rackKP, timeoutMS);
        armRack.config_kI(loopIDX, rackKI, timeoutMS);
        armRack.config_kD(loopIDX, rackKD, timeoutMS);
        armRack.config_kF(loopIDX, rackKF, timeoutMS);

        // Enable Compressor
        armPH.enableCompressorAnalog(100, 120); // Need to evaluate what we want here

    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void goPivotToPosition(double position, boolean on) {
        if(on) {
            armPivot.set(ControlMode.Position, position);
        }
        else {
            armPivot.set(ControlMode.PercentOutput, 0);
        }
    }

    public void goRackToPosition(double position, boolean on) {
        if(on) {
            armRack.set(ControlMode.Position, position);
        }
        else {
            armRack.set(ControlMode.PercentOutput, 0); 
        }
    }

    public double getPivotPosition() {
        return armPivot.getSelectedSensorPosition();
    }

    public double getRackPosition() {
        return armRack.getSelectedSensorPosition();
    }

}
