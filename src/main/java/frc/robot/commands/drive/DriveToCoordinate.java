package frc.robot.commands.drive;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.CustomDriveDistanceCommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * @brief This command is used to drive a specific distance
 *        It will drive straight from wherever the robot is currently facing.
 */
public class DriveToCoordinate extends Command {

    private final DriveSubsystem m_robotDrive;
    private int stallCounter = 0;

    // Control the motion profile for the auto-commands for driving. This is kind-of
    // like a path following
    private final TrapezoidProfile.Constraints m_constraints;
    private final ProfiledPIDController m_controller_x;
    private final ProfiledPIDController m_controller_y;
    private Pose2d m_endPose;
  
     public DriveToCoordinate(DriveSubsystem robotDrive, Pose2d pose) {
        this(robotDrive, pose, new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond/2, DriveConstants.kMaxAccelerationMetersPerSecondSquared/2));

    }

    public DriveToCoordinate(DriveSubsystem robotDrive, Pose2d pose, TrapezoidProfile.Constraints constraints) {
        m_endPose = pose;
        m_constraints = constraints;
        
        m_robotDrive = robotDrive;

        m_controller_x = new ProfiledPIDController(
            CustomDriveDistanceCommandConstants.kPidValues.p, CustomDriveDistanceCommandConstants.kPidValues.i,
            CustomDriveDistanceCommandConstants.kPidValues.d, m_constraints,
            Constants.kDt);
         m_controller_y = new ProfiledPIDController(
                CustomDriveDistanceCommandConstants.kPidValues.p, CustomDriveDistanceCommandConstants.kPidValues.i,
                CustomDriveDistanceCommandConstants.kPidValues.d, m_constraints,
                Constants.kDt);

        addRequirements(robotDrive); // this effectively locks out the joystick

    }

    @Override
    public void initialize() {

        var alliance = DriverStation.getAlliance();
        //guard against double flipping - if pos < 8.5, we are on blue, otherwise already on red
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red && m_endPose.getX() < 8.5) {
            m_endPose = FlippingUtil.flipFieldPose(m_endPose);
        }
        
        InitMotionProfile(m_endPose);
        
        stallCounter = 0;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0, 0, 0, true);
        m_robotDrive.setJoystickTranslateLockout(false);
    }

    @Override
    public void execute() {
        double pid_valX = m_controller_x.calculate(m_robotDrive.getPose().getX());
        double vel_pid_valX = pid_valX * DriveConstants.kMaxSpeedMetersPerSecond;

        double pid_valY = m_controller_y.calculate(m_robotDrive.getPose().getY());
        double vel_pid_valY = pid_valY * DriveConstants.kMaxSpeedMetersPerSecond;

        double finalVelX = m_controller_x.getSetpoint().velocity + vel_pid_valX;
        double finalVelY = m_controller_y.getSetpoint().velocity + vel_pid_valY;

        m_robotDrive.drive(finalVelX, finalVelY, 0, true, false);
    }

    @Override
    public boolean isFinished() {

        //if we are closer than .05 at either target, increment the stall counter
        if (Math.abs(m_robotDrive.getPose().getX() - m_endPose.getX()) < .05 || Math.abs(m_robotDrive.getPose().getY() - m_endPose.getY()) < .05) {
            stallCounter++;
        } else {
            stallCounter = 0;
        }

        return (m_controller_x.atGoal() && m_controller_y.atGoal()) || stallCounter > 15;
    }


    /**
     * @brief Initialize the motion profile for the PID controller
     * @param setpointX
     * @param setpointY
     */
    private void InitMotionProfile(Pose2d setpoint) {

        m_controller_x.reset(m_robotDrive.getPose().getX());
        m_controller_x.setTolerance(.02);
        m_controller_x.setGoal(new TrapezoidProfile.State(setpoint.getX(), 0));

        m_controller_y.reset(m_robotDrive.getPose().getY());
        m_controller_y.setTolerance(.02);
        m_controller_y.setGoal(new TrapezoidProfile.State(setpoint.getY(), 0));

        m_robotDrive.setJoystickTranslateLockout(true);// begin locking out the 0 values from the joystick

    }

}
