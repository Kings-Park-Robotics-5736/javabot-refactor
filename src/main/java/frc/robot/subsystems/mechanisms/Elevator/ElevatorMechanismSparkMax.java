package frc.robot.subsystems.mechanisms.Elevator;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.MotionProfileConstants;
import frc.robot.utils.Types.PidConstants;

public class ElevatorMechanismSparkMax extends ElevatorMechanism {

    private final SparkMax m_leader;
    private final SparkMax m_follower;

    private SparkMaxConfig m_leaderMotorConfig;
    private SparkMaxConfig m_followerMotorConfig;    

    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_encoder;
    
    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RPM.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;

    /**
     * Constructor for ElevatorMechanismCTRE
     * 
     * @param name                 of the elevator mechanism
     * @param primaryMotorId       id of the leader motor
     * @param followerMotorId      id of the follower moter, or NULL if no follower
     * @param pidValues            pid constants
     * @param ffValues             feed forward constants
     * @param limits               limits for the elevator mechanism
     * @param diffThreshold        threshold for detecting if we are done - in
     *                             rotations per second
     * @param staleThreshold       threshold for detecting stale readings - in
     *                             number of periodic cycles
     * @param staleTolerance       threshold for detecting stale readings - in
     *                             rotations per second
     * 
     * @param motionProfileConstants motion profile constants
     */
    public ElevatorMechanismSparkMax(String name,
            Byte primaryMotorId,
            Byte followerMotorId,
            PidConstants pidValues,
            FeedForwardConstants ffValues,
            Limits limits,
            double diffThreshold,
            int staleThreshold,
            double staleTolerance,
            MotionProfileConstants motionProfileConstants) {

        super(limits, name, pidValues, ffValues, diffThreshold, staleThreshold, staleTolerance);


        m_leader = new SparkMax(primaryMotorId, MotorType.kBrushless);
        if(followerMotorId != null){
            m_follower = new SparkMax(followerMotorId, MotorType.kBrushless);
        }else{
            m_follower = null;
            m_followerMotorConfig = null;
        }

        m_leaderMotorConfig = new SparkMaxConfig();
        m_closedLoopController = m_leader.getClosedLoopController();
        m_encoder = m_leader.getEncoder();

        m_leaderMotorConfig.idleMode(IdleMode.kBrake);

        m_leaderMotorConfig.closedLoop
        .pid(pidValues.p, pidValues.i, pidValues.d)
        .outputRange(-1, 1).feedForward.svag(ffValues.ks, ffValues.kv, ffValues.ka, ffValues.kg);

        m_leaderMotorConfig.closedLoop.maxMotion
        .cruiseVelocity(motionProfileConstants.kMaxVelocity)
        .maxAcceleration(motionProfileConstants.kMaxAcceleration)
        .allowedProfileError(motionProfileConstants.maxError); 

        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);

        if(m_follower != null){
            m_followerMotorConfig = new SparkMaxConfig();
            m_followerMotorConfig.idleMode(IdleMode.kBrake);
            m_followerMotorConfig.follow(m_leader);
            SparkMaxUtils.ApplySparkMaxConfig(m_follower, m_followerMotorConfig);
        }

         m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(2), null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                            m_leader.set(-volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Elevator"))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                        getElevatorPosition(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                getRotationsPerMinute(),
                                                    RPM));

                        },
                        this));
    }

    @Override
    public void setSpeed(double speed) {
        m_leader.set(speed);
    }

    @Override
    public double getElevatorPosition() {
        return m_encoder.getPosition();
    }

    @Override
    public double getRotationsPerSecond(){
        return getRotationsPerMinute() / 60; //Sparkmax returns natively in RPM
    }

    public double getRotationsPerMinute(){
        return m_encoder.getVelocity(); //Sparkmax returns natively in RPM
    }

    @Override
    public void RunElevator() {
        m_closedLoopController.setSetpoint(m_targetPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    @Override 
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }


    @Override 
    public void updatePIDGV(PidConstants pid, double g, double v){
        m_leaderMotorConfig.closedLoop
        .p(pid.p)
        .i(pid.i)
        .d(pid.d);
        m_leaderMotorConfig.closedLoop.feedForward.kV(v);
        m_leaderMotorConfig.closedLoop.feedForward.kG(g);
        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); 
    }
}
