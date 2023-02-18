package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ArmSubsystem extends SubsystemBase {
    private static final double PIVOTCONST = -975.64444444;
    private static final double RACKCONST = 15979.6178;

    /* Arm Setup */
    private WPI_TalonFX armPivot;
    private WPI_TalonFX armRack;
    int loopIDX = Constants.ArmConstants.loopIDX;
    int timeoutMS = Constants.ArmConstants.timeoutMS;
    double pivotKP = Constants.ArmConstants.armPivotKP;
    double pivotKI = Constants.ArmConstants.armPivotKI;
    double pivotKD = Constants.ArmConstants.armPivotKD;
    double pivotKF = Constants.ArmConstants.armPivotKF;
    double rackKP = Constants.ArmConstants.armRackKP;
    double rackKI = Constants.ArmConstants.armRackKI;
    double rackKD = Constants.ArmConstants.armRackKI;
    double rackKF = Constants.ArmConstants.armRackKF;
    boolean vacOn = false;

    /* Pneuumatics Setup */
    public int airSupplyCAN = Constants.ArmConstants.airSupplyCAN;
    private PneumaticHub armPH;
    Solenoid vacSolPH;
    

    /* Arm Absolute Encoder? */

    public ArmSubsystem() {
        armPivot = new WPI_TalonFX(Constants.ArmConstants.armPivot);
        armRack = new WPI_TalonFX(Constants.ArmConstants.armRack);

        // Pnumatics init
        armPH = new PneumaticHub(airSupplyCAN);
        vacSolPH = armPH.makeSolenoid(0);
                
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
        armPH.enableCompressorAnalog(110, 120);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Solenoid Value", vacSolPH.get());
        SmartDashboard.putNumber("Compressor PSI", armPH.getPressure(0));
        SmartDashboard.putNumber("Compressor Voltage", armPH.getAnalogVoltage(0));
        SmartDashboard.putNumber("Pivot enc", armPivot.getSelectedSensorPosition());
        SmartDashboard.putNumber("Rack enc", armRack.getSelectedSensorPosition());
        vacSolPH.set(vacOn);
    }

    public void enableVac(boolean on) {
        this.vacOn = on;
    }
    
    public void goPivotToPosition(double degrees) {
        armPivot.set(ControlMode.Position, (degrees-Constants.ArmConstants.pivotBottomAngle)/PIVOTCONST);
    }

    public void goRackToPosition(double inches) {
        armRack.set(ControlMode.Position, inches/RACKCONST);
    }

    public double getPivotPosition() {
        return armPivot.getSelectedSensorPosition();
    }

    public double getRackPosition() {
        return armRack.getSelectedSensorPosition();
    }

}
