package frc.robot.subsystems.mechanisms.Wheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
import frc.robot.utils.Types.MotionProfileConstants;
import frc.robot.utils.Types.PidConstants;

public class WheelMechanismSparkMax extends WheelMechanism {

    private final SparkMax m_follower;
    private final SparkMax m_leader;

    private SparkMaxConfig m_leaderMotorConfig;
    private SparkMaxConfig m_followerMotorConfig;
    private SparkClosedLoopController m_controller;

    private RelativeEncoder m_encoder;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RPM.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;

    /**
     * Constructor for WheelMechanismSparkMax
     * 
     * @param name            of the wheel mechanism
     * @param primaryMotorId  id of the leader motor
     * @param followerMotorId id of the follower motor, or NULL if no follower
     * @param pid             pid constants
     * @param ff              feed forward constants
     * @param diffThreshold   threshold for detecting if we are done - in
     *                        rotations per second
     * @param staleThreshold  threshold for detecting stale readings - in
     *                        number of periodic cycles
     * @param staleTolerance  threshold for detecting stale readings - in
     *                        rotations per second
     * @param motionConstants motion profile constants
     */
    public WheelMechanismSparkMax(String name,
            Byte primaryMotorId,
            Byte followerMotorId,
            PidConstants pidValues,
            FeedForwardConstants ffValues,
            double diffThreshold,
            int staleThreshold,
            double staleTolerance,
            MotionProfileConstants motionConstants) {

        super(name, pidValues, ffValues, diffThreshold, staleThreshold, staleTolerance);

        m_leader = new SparkMax(primaryMotorId, MotorType.kBrushless);
        if (followerMotorId != null) {
            m_follower = new SparkMax(followerMotorId, MotorType.kBrushless);
        } else {
            m_follower = null;
            m_followerMotorConfig = null;
        }

        m_leaderMotorConfig = new SparkMaxConfig();
        m_controller = m_leader.getClosedLoopController();
        m_encoder = m_leader.getEncoder();

        m_leaderMotorConfig.idleMode(IdleMode.kCoast);

        m_leaderMotorConfig.closedLoop
                .pid(pidValues.p, pidValues.i, pidValues.d)
                .outputRange(-1, 1).feedForward.sva(ffValues.ks, ffValues.kv, ffValues.ka);

        m_leaderMotorConfig.closedLoop.maxMotion
                .cruiseVelocity(motionConstants.kMaxVelocity)
                .maxAcceleration(motionConstants.kMaxAcceleration)
                .allowedProfileError(motionConstants.maxError);

        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);

        if (m_follower != null) {
            m_followerMotorConfig = new SparkMaxConfig();
            m_followerMotorConfig.idleMode(IdleMode.kCoast);
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
                            log.motor("Wheel")
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                            m_encoder.getPosition(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    getWheelSpeedRPM(),
                                                    RPM));

                        },
                        this));

    }

    @Override
    public void setSpeed(double speed) {
        m_leader.set(speed);
    }

    public double getWheelSpeedRPS() {
        return getWheelSpeedRPM() / 60.0; // Sparkmax returns natively in RPM
    }

    public double getWheelSpeedRPM() {
        return m_encoder.getVelocity(); // Sparkmax returns natively in RPM
    }

    protected void RunWheel() {
        m_controller.setSetpoint(m_targetSpeedRPS * 60, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void updatePIDGV(PidConstants pid, double g, double v) {
        m_leaderMotorConfig.closedLoop
                .p(pid.p)
                .i(pid.i)
                .d(pid.d);
        m_leaderMotorConfig.closedLoop.feedForward.kV(v);
        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
