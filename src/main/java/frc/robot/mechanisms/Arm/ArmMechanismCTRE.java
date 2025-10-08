package frc.robot.mechanisms.Arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;
import edu.wpi.first.math.util.Units;

public class ArmMechanismCTRE extends ArmMechanism {

    private final TalonFX m_leader;
    private TalonFXConfiguration configs;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    /********************************************************
     * SysId variables
     ********************************************************/
   
    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    public ArmMechanismCTRE(Limits limits, String name, PidConstants pidValues, FeedForwardConstants ffValues) {
        super(limits, name, pidValues, ffValues);
        m_leader = new TalonFX(ArmConstants.kMotorID, ArmConstants.kCanName);

        configs = new TalonFXConfiguration();

        //all these values must be in terms of unit of rotations and seconds, NOT radians
        configs.Slot0.kP = ArmConstants.kPidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = ArmConstants.kPidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = ArmConstants.kPidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kA = ArmConstants.kFFValues.ka;
        configs.Slot0.kG = ArmConstants.kFFValues.kg;
        configs.Slot0.kV = ArmConstants.kFFValues.kv;
        configs.Slot0.kS = ArmConstants.kFFValues.ks;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        // Note that the sensor offset and ratios must be configured so that the
        //sensor reports a position of 0 when the mechanism is horizonal
        //(parallel to the ground), and the reported sensor position is 1:1 with
        //the mechanism.
        configs.Feedback.SensorToMechanismRatio = 72.73;

        configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kMaxVelocity;// 2* Math.PI;
        configs.MotionMagic.MotionMagicAcceleration = ArmConstants.kMaxAcceleration; //4* Math.PI; 
        configs.MotionMagic.MotionMagicJerk = ArmConstants.kMaxJerk; //20*Math.PI; 

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the + Arm. Restart robot!");
        }

        m_leader.setNeutralMode(NeutralModeValue.Brake);
       
        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null, state->SignalLogger.writeString("arm-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            m_leader.setControl(m_sysidControl.withOutput(volts));
                        },
                     null,
                        this)); 


        BaseStatusSignal.setUpdateFrequencyForAll(250,m_leader.getPosition(), m_leader.getVelocity(), m_leader.getMotorVoltage());
        m_leader.optimizeBusUtilization();
        //SignalLogger.start();
    }

    //add all stubs for required overrides
    public void setSpeed(double speed){
        m_leader.set(speed);
    }
    public double getArmPositionRadians(){
        //convert from rotations to radians
        return Units.rotationsToRadians(m_leader.getPosition().refresh().getValueAsDouble());
    }
    protected double getArmPositionRadiansPerSecond(){
        //convert from rotations per second to radians per second
        return Units.rotationsToRadians(m_leader.getVelocity().refresh().getValueAsDouble());

    }
    protected void resetEncoder(){
        m_leader.setPosition(0);
    }

    public void setInitialPositionRotations(double position){
        m_leader.setPosition(position); //this is the arm offset value
    }

    /**
     * @brief Initializes a motion to a given position in radians
     * @param position The target position in radians
     * @note This does not start the motion, it just sets the target position. The motion will be run in the periodic method
     */
    protected void InitMotion(double position){
        double sanitizedSetpoint = sanitizePositionSetpoint(position);
        m_targetPosition = sanitizedSetpoint;
        manualControl = false;
        SmartDashboard.putNumber("Arm setpoint" + m_name, m_targetPosition);
    }

    protected void RunArm(){
        m_leader.setControl(m_request.withPosition(Units.radiansToRotations(m_targetPosition)));
    }

    @Override 
    public void updatePIDGV(PidConstants pid, double g, double v){
        configs.Slot0.kP = pid.p; 
        configs.Slot0.kI = pid.i; 
        configs.Slot0.kD = pid.d;
        configs.Slot0.kG = g;
        configs.Slot0.kV = v;
        TalonUtils.ApplyTalonConfig(m_leader, configs);
    }

    private double sanitizePositionSetpoint(double setpoint){
        if (setpoint > ArmConstants.kLimits.high){
            setpoint = ArmConstants.kLimits.high;
        }
        if(setpoint < ArmConstants.kLimits.low){
            setpoint = ArmConstants.kLimits.low;
        }
        return setpoint;
    }


    
}
