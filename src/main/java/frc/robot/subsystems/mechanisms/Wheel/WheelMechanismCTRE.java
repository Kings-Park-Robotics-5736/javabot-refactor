package frc.robot.subsystems.mechanisms.Wheel;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.MotionProfileConstants;
import frc.robot.utils.Types.PidConstants;

public class WheelMechanismCTRE extends WheelMechanism {

    private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

    private final TalonFX m_leader;
    private final TalonFX m_follower;

    private TalonFXConfiguration configs;

    /********************************************************
     * SysId variables
     ********************************************************/

    private final SysIdRoutine m_sysIdRoutine;
    private final VoltageOut m_sysidControl = new VoltageOut(0);

    /**
     * Constructor for WheelMechanismCTRE
     * 
     * @param name                 of the wheel mechanism
     * @param primaryMotorId       id of the leader motor
     * @param followerMotorId      id of the follower moter, or NULL if no follower
     * @param canName              name of the can bus for both motors.
     * @param pidValues            pid constants
     * @param ffValues             feed forward constants
     * @param diffThreshold        threshold for detecting if we are done - in
     *                             rotations per second
     * @param staleThreshold       threshold for detecting stale readings - in
     *                             number of periodic cycles
     * @param staleTolerance       threshold for detecting stale readings - in
     *                             rotations per second
     * @param motionMagicConstants motion magic constants
     */
    public WheelMechanismCTRE(String name, Byte primaryMotorId, Byte followerMotorId, String canName,
            PidConstants pidValues, FeedForwardConstants ffValues, double diffThreshold, int staleThreshold,
            double staleTolerance, MotionProfileConstants motionMagicConstants) {

        super(name, pidValues, ffValues, diffThreshold, staleThreshold, staleTolerance);
        m_leader = new TalonFX(primaryMotorId, canName);

        if (followerMotorId != null) {
            m_follower = new TalonFX(followerMotorId, canName);
        } else {
            m_follower = null;
        }

        configs = new TalonFXConfiguration();

        // all these values must be in terms of unit of rotations and seconds, NOT
        // radians
        configs.Slot0.kP = pidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = pidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = pidValues.d; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kA = ffValues.ka;
        configs.Slot0.kV = ffValues.kv;
        configs.Slot0.kS = ffValues.ks;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        configs.MotionMagic.MotionMagicAcceleration = motionMagicConstants.kMaxAcceleration;
        configs.MotionMagic.MotionMagicJerk = motionMagicConstants.kMaxJerk;

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the" + name + "wheel. Restart robot!");
        }

        if (m_follower != null) {
            if (!TalonUtils.ApplyTalonConfig(m_follower, configs)) {
                System.out
                        .println("!!!!!ERROR!!!! Could not initialize the " + name + " FOLLOWER wheel.Restart robot!");
            }

            m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
            m_follower.setNeutralMode(NeutralModeValue.Coast);
        }

        m_leader.setNeutralMode(NeutralModeValue.Coast);

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        state -> SignalLogger.writeString("wheel-state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            m_leader.setControl(m_sysidControl.withOutput(volts));
                        },
                        null,
                        this));

        BaseStatusSignal.setUpdateFrequencyForAll(250, m_leader.getPosition(), m_leader.getVelocity(),
                m_leader.getMotorVoltage());
        m_leader.optimizeBusUtilization();
        // SignalLogger.start();
    }

    public void setSpeed(double speed) {
        m_leader.set(speed);
    }

    public double getWheelSpeedRPS() {
        return m_leader.getVelocity().refresh().getValueAsDouble();
    }

  

    protected void RunWheel() {
        m_leader.setControl(m_request.withVelocity(m_targetSpeedRPS));

    }

    @Override
    public void updatePIDGV(PidConstants pid, double g, double v) {
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
