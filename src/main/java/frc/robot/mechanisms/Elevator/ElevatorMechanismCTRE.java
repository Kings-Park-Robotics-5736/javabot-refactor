package frc.robot.mechanisms.Elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.utils.TalonUtils;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Types.PidConstants;

public class ElevatorMechanismCTRE extends ElevatorMechanism {

    private final TalonFX m_follower;
    private final TalonFX m_leader;

    private TalonFXConfiguration configs;

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    /********************************************************
     * SysId variables
     ********************************************************/
    private final SysIdRoutine m_sysIdRoutine;
    private final MutVoltage m_appliedVoltage = (Volts.mutable(0));
    private final MutAngle m_distance = (Rotations.mutable(0));
    private final MutAngularVelocity m_velocity = (RotationsPerSecond.mutable(0));


    public ElevatorMechanismCTRE(String name, Boolean hasFollower, PidConstants pidValues, FeedForwardConstants ffValues, Limits limits) {
        super(limits, name, pidValues, ffValues);
        
        m_leader = new TalonFX(ElevatorConstants.kLeaderDeviceId, ElevatorConstants.kCanName);
        if(hasFollower){
            m_follower = new TalonFX(ElevatorConstants.kFollowerDeviceId, ElevatorConstants.kCanName);
        }else{
            m_follower = null;
        }
       
        configs = new TalonFXConfiguration();
        configs.Slot0.kP = pidValues.p; 
        configs.Slot0.kI = pidValues.i; 
        configs.Slot0.kD = pidValues.d;
        configs.Slot0.kA = ffValues.ka;
        configs.Slot0.kG = ffValues.kg;
        configs.Slot0.kV = ffValues.kv;
        configs.Slot0.kS = ffValues.ks;
        configs.Voltage.PeakForwardVoltage = 6;
        configs.Voltage.PeakReverseVoltage = -6;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kMaxVelocity;

        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kMaxAcceleration; 
        configs.MotionMagic.MotionMagicJerk = ElevatorConstants.kMaxJerk; 

        if (!TalonUtils.ApplyTalonConfig(m_leader, configs)) { 
            System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator LEADER. Restart robot!");
        }

        if(m_follower != null){
            if (!TalonUtils.ApplyTalonConfig(m_follower, configs)) { 
                System.out.println("!!!!!ERROR!!!! Could not initialize the Elevator FOLLOWER. Restart robot!");
            }

            m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
        }

        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(null, Volts.of(2), null),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> {
                        // CANNOT use set voltage, it does not work. This normalizes the voltage between
                        // -1 and 0 and 1
                        m_leader.set(volts.in(Volts) / RobotController.getBatteryVoltage());
                    },
                    log -> {
                        log.motor(("Elevator"))
                                .voltage(m_appliedVoltage.mut_replace(
                                        m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                .angularPosition(m_distance.mut_replace(m_leader.getPosition().refresh().getValueAsDouble(),
                                        Rotations))
                                .angularVelocity(
                                        m_velocity.mut_replace(m_leader.getVelocity().refresh().getValueAsDouble(),
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
        return m_leader.getPosition().refresh().getValueAsDouble();
    }

    @Override
    public double getRotationsPerSecond(){
        return m_leader.getVelocity().refresh().getValueAsDouble();
    }

    @Override
    public void RunElevator() {
        m_leader.setControl(m_request.withPosition(m_targetPosition));
    }

    @Override 
    public void resetEncoder() {
        m_leader.setPosition(0);
    }

    @Override
    public void InitMotion(double position){
        m_targetPosition = position;
        staleCounter = 0; //new target, reset stale counter
        SmartDashboard.putNumber("Elevator m_setpoint" + m_name, m_targetPosition);
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
