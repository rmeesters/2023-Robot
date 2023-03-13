package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;


public class ArmSubsystem extends SubsystemBase {
    private static final double PIVOTCONST = 975.64444444;
    private static final double RACKCONST = -6848.4076; // was -15979.6178;

    /* Arm Setup */
    // CANCoder pivotEncoder;
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

    /* Pneuumatics Setup */
    public int airSupplyCAN = Constants.ArmConstants.airSupplyCAN;
    private PneumaticHub armPH;
    Solenoid vacSolPH,clawSolPH,fanPH;
    SendableChooser<String> colorChooser = new SendableChooser<>();
    String selectedColor;
  

    /* Arm Absolute Encoder? */

    public ArmSubsystem() {
        armPivot = new WPI_TalonFX(Constants.ArmConstants.armPivot);
        armRack = new WPI_TalonFX(Constants.ArmConstants.armRack);
        //pivotEncoder = new CANCoder(Constants.ArmConstants.pivotEncoder); // 5 in Constants

        // Pnumatics init
        armPH = new PneumaticHub(airSupplyCAN);
        vacSolPH = armPH.makeSolenoid(0);
        clawSolPH = armPH.makeSolenoid(1);
        fanPH = armPH.makeSolenoid(3);
                
        // Configure Arm Defaults
        /* pivotEncoder.configFactoryDefault();
        pivotEncoder.clearStickyFaults();
        pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        pivotEncoder.setPosition(Constants.ArmConstants.pivotBottomAngle);
        pivotEncoder.configSensorDirection(false);
        pivotEncoder.configMagnetOffset(0); */

        armPivot.configFactoryDefault();
        armPivot.setSelectedSensorPosition(0);
        armPivot.setNeutralMode(NeutralMode.Brake);
        armRack.configFactoryDefault();
        armRack.setSelectedSensorPosition(0);
        armRack.setNeutralMode(NeutralMode.Brake); 

        

        // Config feedback sensors for PID
        //armPivot.configRemoteFeedbackFilter(pivotEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0);
        //armPivot.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, loopIDX, timeoutMS);
        armPivot.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeoutMS);
        armRack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, loopIDX, timeoutMS);

        // Config peak and nominal outputs
        armPivot.configNominalOutputForward(0, timeoutMS);
        armPivot.configNominalOutputReverse(0, timeoutMS);
        armPivot.configPeakOutputForward(0.5, timeoutMS);
        armPivot.configPeakOutputReverse(-0.5, timeoutMS);
        armPivot.setInverted(true);
        armPivot.setSensorPhase(true);
        armPivot.configMotionAcceleration(PIVOTCONST*20);
        armPivot.configMotionCruiseVelocity(PIVOTCONST*10);

        armRack.configNominalOutputForward(0, timeoutMS);
        armRack.configNominalOutputReverse(0, timeoutMS);
        armRack.configPeakOutputForward(1, timeoutMS);
        armRack.configPeakOutputReverse(-1, timeoutMS);
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

        // Enable Compressor and fan
        armPH.enableCompressorAnalog(110, 120);
        fanPH.set(true);    

    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //TODO clean unused smartdashboard puts
        SmartDashboard.putBoolean("Solenoid Value", vacSolPH.get());
        SmartDashboard.putNumber("Compressor PSI", armPH.getPressure(0));
        SmartDashboard.putNumber("Compressor Voltage", armPH.getAnalogVoltage(0));
        SmartDashboard.putNumber("Compressor Regulated Voltage", armPH.get5VRegulatedVoltage());
        SmartDashboard.putNumber("Pivot enc", getPivotAngle());
        SmartDashboard.putNumber("Rack enc", armRack.getSelectedSensorPosition());
        if(armPH.getPressure(0) > 75.0){
            SmartDashboard.putBoolean("Compressor Pressure", false);
        }
        else{
            SmartDashboard.putBoolean("Compressor Pressure", true);
        }
    }

    public void enableVac(boolean on) {
        vacSolPH.set(on);
    }
    public void toggleVac() {
        vacSolPH.toggle();
    }
    public void enableClaw(boolean on) {
        clawSolPH.set(on);
    }
    public void toggleClaw() {
        clawSolPH.toggle();
    }
    
    public void goPivotToPosition(double degrees) {
        armPivot.set(ControlMode.MotionMagic, (degrees-Constants.ArmConstants.pivotBottomAngle)*PIVOTCONST);
    }

    public void goRackToPosition(double inches) {
        armRack.set(ControlMode.Position, inches*RACKCONST);
    }

    public void manualLiftArm(double number){
        armPivot.set(ControlMode.PercentOutput, number);
    }

    
    

    public double getPivotAngle() {
        return (armPivot.getSelectedSensorPosition()/PIVOTCONST)+Constants.ArmConstants.pivotBottomAngle;
    }

    public double getRackPosition() {
        return armRack.getSelectedSensorPosition()/RACKCONST;
    }

}
