package frc.robot.subsystems.mechanisms.Arm;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.MotionProfileConstants;
import frc.robot.utils.Types.PidConstants;

public class ArmMechanismCTRE extends ArmMechanism {

    private final TalonFX m_leader;
    private final TalonFX m_follower;

    private TalonFXConfiguration configs;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    /********************************************************
     * SysId variables
     ********************************************************/
   
    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    /**
     * Constructor for ArmMechanismCTRE
     * 
     * @param name                 of the arm mechanism
     * @param primaryMotorId       id of the leader motor
     * @param followerMotorId      id of the follower moter, or NULL if no follower
     * @param canName              name of the can bus for both motors.
     * @param pidValues            pid constants
     * @param ffValues             feed forward constants
     * @param limits               limits for the arm mechanism
     * @param diffThreshold        threshold for detecting if we are done - in
     *                             rotations per second
     * @param staleThreshold       threshold for detecting stale readings - in
     *                             number of periodic cycles
     * @param staleTolerance       threshold for detecting stale readings - in
     *                             rotations per second
     * @param sensorToMechRatio    sensor to mechanism ratio for MagicMotion
     * @apiNote                    For CTRE, this should be > 1 if 1 turn of the encoder is < 1 turn of the mechanism

     * @param motionMagicConstants motion magic constants
     */
    public ArmMechanismCTRE(String name,
            Byte primaryMotorId,
            Byte followerMotorId,
            String canName,
            PidConstants pidValues,
            FeedForwardConstants ffValues,
            Limits limits,
            double diffThreshold,
            int staleThreshold,
            double staleTolerance,
            double sensorToMechRatio,
            MotionProfileConstants motionMagicConstants) {

        super(limits, name, pidValues, ffValues, diffThreshold, staleThreshold, staleTolerance);
        m_leader = new TalonFX(primaryMotorId, canName);

        if(followerMotorId !=  null){
            m_follower = new TalonFX(followerMotorId, canName);
        }else{
            m_follower = null;
        }

        configs = new TalonFXConfiguration();

        //all these values must be in terms of unit of rotations and seconds, NOT radians
        configs.Slot0.kP = pidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = pidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = pidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kA = ffValues.ka;
        configs.Slot0.kG = ffValues.kg;
        configs.Slot0.kV = ffValues.kv;
        configs.Slot0.kS = ffValues.ks;
        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        // Note that the sensor offset and ratios must be configured so that the
        //sensor reports a position of 0 when the mechanism is horizonal
        //(parallel to the ground), and the reported sensor position is 1:1 with
        //the mechanism.
        configs.Feedback.SensorToMechanismRatio = sensorToMechRatio;

        configs.MotionMagic.MotionMagicCruiseVelocity = motionMagicConstants.kMaxVelocity;// 2* Math.PI;
        configs.MotionMagic.MotionMagicAcceleration = motionMagicConstants.kMaxAcceleration; //4* Math.PI; 
        configs.MotionMagic.MotionMagicJerk = motionMagicConstants.kMaxJerk; //20*Math.PI; 

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Arm. Restart robot!");
        }

        if(m_follower != null){
            if (!TalonUtils.ApplyTalonConfig(m_follower, configs)) { 
                System.out.println("!!!!!ERROR!!!! Could not initialize the Arm FOLLOWER. Restart robot!");
            }

            m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
            m_follower.setNeutralMode(NeutralModeValue.Brake);
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

   


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    
}
