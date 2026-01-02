package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

/**
 * @brief Helper utilities that are common across the various drive routines
 */
public class DriveCommandsCommon {

    /**
     * @brief Given a heading and a degree of a target off from the center,
     *        calculate how much we need to rotate, and return the velocity of the
     *        theta controller
     * 
     * @param heading              The current robot heading
     * @param useCameraMeasurement The camera might not be updating fast enough (low
     *                             frame rate). This means that we might be using a
     *                             stale value. If so, don't use the angle to
     *                             calculate a new goal; rather, keep going to old
     *                             goal.
     * @param degPi                The reported angle of the target (i.e. from a
     *                             raspberryPi camera)
     * @param m_controller_theta   The motion controller for theta. It computes the
     *                             profile.
     * 
     * @return Return the velocity, in rad/s to send to drive() (as if it came from
     *         a joystick). -100 if no target is found from the source image
     */
    public static double calculateRotationToTarget(double heading, boolean useCameraMeasurement, double degPi,
            ProfiledPIDController m_controller_theta) {

        double finalVelTheta = -100;
        if (degPi > -1000) {

            //Get the output of the PID calculation first
            final double turnOutput;
            if (useCameraMeasurement) {
                turnOutput = m_controller_theta.calculate(heading,
                        heading - Units.degreesToRadians(degPi));
            } else {
                turnOutput = m_controller_theta.calculate(heading);
            }

            //calculate the veloicty as a result of the PID calculation
            double vel_pid_theta = turnOutput * DriveConstants.kMaxSpeedMetersPerSecond;

            //factor in the velocity from the motion profile
            finalVelTheta = m_controller_theta.getSetpoint().velocity + vel_pid_theta;

        }
        return finalVelTheta;
    }

    /**
     * @brief Given a heading and a desired rotation, calculate the velocity of the
     *       theta controller
     * @param heading    the current robot heading
     * @param desiredRot the desired robot heading
     * @param m_controller_theta
     * @return the velocity, in rad/s to send to drive() (as if it came from a joystick)
     */
    public static double calculateRotationToFieldPos(double heading, double desiredRot,
            ProfiledPIDController m_controller_theta) {

        final double turnOutput;
        turnOutput = m_controller_theta.calculate(heading, desiredRot);
       
        double vel_pid_theta = turnOutput * DriveConstants.kMaxSpeedMetersPerSecond;

        double finalVelTheta = m_controller_theta.getSetpoint().velocity + vel_pid_theta;

        return finalVelTheta;
    }
}
