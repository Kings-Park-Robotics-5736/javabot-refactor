package frc.robot.mechanisms.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Types.PidConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.SparkMaxUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorMechanismSparkMax extends ElevatorMechanism {

    private final SparkMax m_follower;
    private final SparkMax m_leader;

    private SparkMaxConfig m_leaderMotorConfig;
    private SparkMaxConfig m_followerMotorConfig;    

    private SparkClosedLoopController m_closedLoopController;
    private RelativeEncoder m_encoder;

    private ElevatorFeedforward m_feedforward;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_prior_iteration_setpoint;



    
    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));
    private final SysIdRoutine m_sysIdRoutine;

    public ElevatorMechanismSparkMax(String name, Boolean hasFollower, PidConstants pidValues, FeedForwardConstants ffValues, Limits limits) {
        super(limits, name, pidValues, ffValues);  
        
        m_leader = new SparkMax(ElevatorConstants.kLeaderDeviceId, MotorType.kBrushless);
        if(hasFollower){
            m_follower = new SparkMax(ElevatorConstants.kFollowerDeviceId, MotorType.kBrushless);
        }else{
            m_follower = null;
            m_followerMotorConfig = null;
        }

        m_leaderMotorConfig = new SparkMaxConfig();
       

        m_leaderMotorConfig.idleMode(IdleMode.kBrake);

        m_leaderMotorConfig.closedLoop

        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(pidValues.p)
        .i(pidValues.i)
        .d(pidValues.d)
        .iZone(0)
        .velocityFF(0)
        .outputRange(-1, 1);

       

        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);

        if(m_follower != null){
            m_followerMotorConfig = new SparkMaxConfig();
            m_followerMotorConfig.idleMode(IdleMode.kBrake);
            m_followerMotorConfig.follow(m_leader);
            SparkMaxUtils.ApplySparkMaxConfig(m_follower, m_followerMotorConfig);
        }


        m_closedLoopController = m_leader.getClosedLoopController();

        m_encoder = m_leader.getEncoder();

        m_feedforward = new ElevatorFeedforward(ffValues.ks, ffValues.kg, ffValues.kv, ffValues.ka);

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
                                                    getRotationsPerSecond(),
                                                    RotationsPerSecond));

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
        return m_encoder.getVelocity() / 60; //Sparkmax returns natively in RPM
    }

    @Override
    public void RunElevator() {
        m_prior_iteration_setpoint = m_profile.calculate(0.02, m_prior_iteration_setpoint,new TrapezoidProfile.State(m_targetPosition, 0));
        double ff = m_feedforward.calculate(m_prior_iteration_setpoint.position, m_prior_iteration_setpoint.velocity);
        m_closedLoopController.setReference(m_prior_iteration_setpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff,
                ArbFFUnits.kVoltage);
    }

    @Override 
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    @Override
    public void InitMotion(double position){
        m_targetPosition = position;
        staleCounter = 0; //new target, reset stale counter
        m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));
        m_prior_iteration_setpoint = new TrapezoidProfile.State(getElevatorPosition(), 0);
        SmartDashboard.putNumber("Elevator m_setpoint" + m_name, m_targetPosition);
    }

    @Override 
    public void updatePIDGV(PidConstants pid, double g, double v){
        m_leaderMotorConfig.closedLoop
        .p(pid.p)
        .i(pid.i)
        .d(pid.d);
        m_feedforward = new ElevatorFeedforward(m_feedforward.getKs(), g, v, m_feedforward.getKa());
        SparkMaxUtils.ApplySparkMaxConfig(m_leader, m_leaderMotorConfig);
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); 
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); 
    }
}
