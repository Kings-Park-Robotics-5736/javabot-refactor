package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.Elevator.ElevatorMechanism;
import frc.robot.subsystems.mechanisms.Elevator.ElevatorMechanismCTRE;
import frc.robot.subsystems.mechanisms.Elevator.ElevatorMechanismSparkMax;
import frc.robot.Constants.ElevatorConstants;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

    private ElevatorMechanism m_elevatorMechanism;

    public ElevatorSubsystem() {
        m_elevatorMechanism = new ElevatorMechanismSparkMax(
                "Elevator",
                ElevatorConstants.kLeaderDeviceId,
                ElevatorConstants.kFollowerDeviceId,
                ElevatorConstants.kPidValues,
                ElevatorConstants.kFFValues,
                ElevatorConstants.kLimits,
                ElevatorConstants.kDiffThreshold,
                ElevatorConstants.kStaleThreshold,
                ElevatorConstants.kStaleTolerance,
                ElevatorConstants.kMotionProfileConstants
                );

        /*
        m_elevatorMechanism = new ElevatorMechanismCTRE(
                    "Elevator",
                    ElevatorConstants.kLeaderDeviceId,
                    ElevatorConstants.kFollowerDeviceId,
                    "rio",
                    ElevatorConstants.kPidValues,
                    ElevatorConstants.kFFValues,
                    ElevatorConstants.kLimits,
                    ElevatorConstants.kDiffThreshold,
                    ElevatorConstants.kStaleThreshold,
                    ElevatorConstants.kStaleTolerance,
                    ElevatorConstants.kMotionProfileConstants
                    );
        */
    }

    /************************************************************************
     * Pass through commands for the elevator mechanisms
     ************************************************************************/
    public Command RunElevatorToPositionCommand(double setpoint, Boolean async) {
        return m_elevatorMechanism.RunElevatorToPositionCommand(setpoint, async);
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier speedSupplier) {
        return m_elevatorMechanism.RunElevatorManualSpeedCommand(speedSupplier);
    }

    public Command RunElevatorToPositionCommandEarlyFinish(double setpoint) {
        return m_elevatorMechanism.RunElevatorToPositionCommandEarlyFinish(setpoint);
    }

    /************************************************************************
     * Custom commands for this robot's elevator
     ************************************************************************/

}
