package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.Arm.ArmMechanism;
import frc.robot.subsystems.mechanisms.Arm.ArmMechanismCTRE;
import frc.robot.subsystems.mechanisms.Arm.ArmMechanismSparkMax;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private ArmMechanism m_armMechanism;

    public ArmSubsystem() {
        m_armMechanism = new ArmMechanismSparkMax(
                "Arm",
                ArmConstants.kMotorID,
                null,
                ArmConstants.kPidValues,
                ArmConstants.kFFValues,
                ArmConstants.kLimits,
                ArmConstants.kDiffThreshold,
                ArmConstants.kStaleThreshold,
                ArmConstants.kStaleTolerance,
                ArmConstants.kSensorToMechanismRatio,
                ArmConstants.kMotionProfileConstants
                );

        
        /*
        m_armMechanism = new ArmMechanismCTRE(
                    "Arm",
                    ArmConstants.kMotorID,
                    null,
                    "rio",
                    ArmConstants.kPidValues,
                    ArmConstants.kFFValues,
                    ArmConstants.kLimits,
                    ArmConstants.kDiffThreshold,
                    ArmConstants.kStaleThreshold,
                    ArmConstants.kStaleTolerance,
                    ArmConstants.kSensorToMechanismRatio,
                    ArmConstants.kMotionProfileConstants
                    );
        */

       
    }

    /************************************************************************
     * Pass through commands for the arm mechanisms
     ************************************************************************/
    public Command RunArmToPositionCommand(double setpoint, Boolean async) {
        return m_armMechanism.RunArmToPositionCommand(setpoint, async);
    }

    public Command RunArmManulSpeedCommand(DoubleSupplier speedSupplier) {
        return m_armMechanism.RunArmManulSpeedCommand(speedSupplier);
    }

    public Command RunArmToPositionCommandEarlyStop(double setpoint) {
        return m_armMechanism.RunArmToPositionCommandEarlyStop(setpoint);
    }

    /************************************************************************
     * Custom commands for this robot's arm
     ************************************************************************/

}
