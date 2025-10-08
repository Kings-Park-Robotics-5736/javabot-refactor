package frc.robot.mechanisms.Arm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

public class ArmMechanismSparkMax extends ArmMechanism {

    private final SparkMax m_leader;

    private SparkMaxConfig m_leaderMotorConfig;

    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_encoder;

    private ArmFeedforward m_feedforward;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;

    public ArmMechanismSparkMax(String name, Boolean hasFollower, PidConstants pidValues, FeedForwardConstants ffValues, Limits limits) {
        super(limits, name, pidValues, ffValues);

        m_leader = new SparkMax(ArmConstants.kMotorID, MotorType.kBrushless);
        

        m_leaderMotorConfig = new SparkMaxConfig();
        m_leaderMotorConfig.idleMode(IdleMode.kBrake);

        m_leaderMotorConfig.closedLoop
                .p(pidValues.p)
                .i(pidValues.i)
                .d(pidValues.d)
                .iZone(0)
                .velocityFF(0)
                .outputRange(-1, 1);

        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);

       
        m_closedLoopController = m_leader.getClosedLoopController();
        m_encoder = m_leader.getEncoder();

        m_feedforward = new ArmFeedforward(ffValues.ks, ffValues.kg, ffValues.kv, ffValues.ka);

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(2), null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            m_leader.set(-volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor("Arm")
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(
                                            getArmPositionRadians(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    getArmPositionRadiansPerSecond(),
                                                    RotationsPerSecond));
                        },
                        this));
    }

    @Override
    public void setSpeed(double speed) {
        m_leader.set(speed);
    }

    @Override
    public double getArmPositionRadians() {
        return Units.rotationsToRadians(m_encoder.getPosition());
    }

    @Override
    protected double getArmPositionRadiansPerSecond() {
        return Units.rotationsToRadians(m_encoder.getVelocity() / 60); // Convert RPM to rotations per second, then to radians per second
    }

    @Override
    protected void resetEncoder() {
        m_encoder.setPosition(0);
    }

    public void setInitialPositionRotations(double position) {
        m_encoder.setPosition(position);
    }

    @Override
    protected void InitMotion(double position) {
        double sanitizedSetpoint = sanitizePositionSetpoint(position);
        m_targetPosition = sanitizedSetpoint;
        manualControl = false;
        staleCounter = 0; // new target, reset stale counter
        m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getArmPositionRadians(), 0);
        SmartDashboard.putNumber("Arm setpoint" + m_name, m_targetPosition);
    }

    @Override
    protected void RunArm() {
        m_prior_iteration_setpoint = m_profile.calculate(0.02, m_prior_iteration_setpoint,
                new TrapezoidProfile.State(m_targetPosition, 0));
        double ff = m_feedforward.calculate(m_prior_iteration_setpoint.position, m_prior_iteration_setpoint.velocity);
        m_closedLoopController.setReference(Units.radiansToRotations(m_prior_iteration_setpoint.position),
                SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, ArbFFUnits.kVoltage);
    }

    @Override
    public void updatePIDGV(PidConstants pid, double g, double v) {
        m_leaderMotorConfig.closedLoop
                .p(pid.p)
                .i(pid.i)
                .d(pid.d);
        m_feedforward = new ArmFeedforward(m_feedforward.getKs(), g, v, m_feedforward.getKa());
        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
    }

    private double sanitizePositionSetpoint(double setpoint) {
        if (setpoint > ArmConstants.kLimits.high) {
            setpoint = ArmConstants.kLimits.high;
        }
        if (setpoint < ArmConstants.kLimits.low) {
            setpoint = ArmConstants.kLimits.low;
        }
        return setpoint;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}