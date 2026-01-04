package frc.robot.commands.drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.CenterToFieldPositionConstants;
import frc.robot.subsystems.phoenixDrive.DriveSubsystem;



/**
 * @brief This command centers the robot to a specific point on the field. It
 *        does NOT drive to it
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be locked out. As such, we will manually lock out the rotate
 *       joystick.
 */
public class  RotateToFaceFieldElementCommand extends CenterToTargetCommand {

    private static final double POSE_ERROR_THRESH = Math.toRadians(4);
    private static double m_goal_rotation = 0;
    private Pose2d m_fieldElementPose;
    private boolean m_matchFieldElementPose;
    private int m_stale_counter;
    private double m_threshold;
    private double stale_value;

    /**
     * @brief This command makes the robot face a specific field element, either matching the pose, or aiming at it.
     * @param robot_drive
     * @param fieldElementPose - the field element pose to face
     * @param matchFieldElementPose - if true, face the same direction as the field element immediately, else face towards it.
     *                                For instance, if a platform is on the side of the field, and fieldElementPose is 90deg, if this is set
     *                                to true, the robot will face 90deg, else it will face towards as you drive to it (at varying angles using the translation X,Y coords)
     * @param infinite
     * @param threshold
     */
    public RotateToFaceFieldElementCommand(DriveSubsystem robot_drive, Pose2d fieldElementPose, boolean matchFieldElementPose, boolean infinite, double threshold) {
       
        // call the parent constructor.
        super(robot_drive, infinite, new TrapezoidProfile.Constraints(
            CenterToFieldPositionConstants.kMaxSpeedMetersPerSecond, CenterToFieldPositionConstants.kMaxAccelerationMetersPerSecondSquared), CenterToFieldPositionConstants.kPidValues); 
        m_fieldElementPose = fieldElementPose;
        m_matchFieldElementPose = matchFieldElementPose;
        m_threshold = threshold;

       // NOTE - we explicitly don't do the below line; else can't drive while
        // centering
        // addRequirements(m_drive);
    }

    public RotateToFaceFieldElementCommand(DriveSubsystem robot_drive, Pose2d fieldElementPose, boolean matchFieldElementPose, boolean infinite) {
        this(robot_drive, fieldElementPose, matchFieldElementPose, infinite,POSE_ERROR_THRESH);
    }

    

    @Override
    public void initialize() {
        m_drive.setJoystickRotateLockout(true, true);
        m_controller_theta.reset(m_drive.getPose().getRotation().getRadians());
        m_controller_theta.setTolerance(0.06);
        m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("----------------Centering to target pose. -----------------------");
        calculateRotation();
        System.out.println("----------------Desired Goal is  " + m_goal_rotation + " -----------------------");
        m_stale_counter = 0;
        stale_value = 0;

    }


    @Override
    protected void centerOnTarget(double angle, boolean useCameraMeasurement) {
        double rotationVel = DriveCommandsCommon.calculateRotationToFieldPos(
                m_drive.getPose().getRotation().getRadians(),
                angle, m_controller_theta);
        m_drive.setRotateLockoutValue(rotationVel);
        m_drive.drive(0, 0, rotationVel, true, false);
    }

    public double calculateRotation(){
        Pose2d robotPose = m_drive.getPose();
        double rotationOffset = m_fieldElementPose.getRotation().getRadians();

        if(m_matchFieldElementPose){
            m_goal_rotation = rotationOffset;
        }else{
            var xDelta = robotPose.getTranslation().getX() - m_fieldElementPose.getTranslation().getX();
            var yDelta = robotPose.getTranslation().getY() - m_fieldElementPose.getTranslation().getY();
            var angleToTarget = Math.atan(yDelta / xDelta) + rotationOffset; // normally tan is x/y, but in frc coords, it
                                                                            // is y / x
            m_goal_rotation = angleToTarget;

        }        
        return m_goal_rotation;
    }

    @Override
    public void execute() {

        // calculate the angle of our current pose to the target
        var angleToTarget = calculateRotation();
        centerOnTarget(angleToTarget, true);

    }

    protected boolean checkTurningDone() {
        
        if(Math.abs(stale_value - m_drive.getPose().getRotation().getRadians() ) < Math.toRadians(.25)){
            m_stale_counter++;
        }else{
            stale_value = m_drive.getPose().getRotation().getRadians();
            m_stale_counter = 0;
        }
        System.out.println("Stale Counter = " + m_stale_counter + ", angle offset = " + Math.abs(m_goal_rotation - m_drive.getPose().getRotation().getRadians() ));
        return Math.abs(m_goal_rotation - m_drive.getPose().getRotation().getRadians() ) < m_threshold || m_stale_counter > 20;
    }

    public static boolean checkTurningDoneStatic(DriveSubsystem drive){
        return Math.abs(m_goal_rotation - drive.getPose().getRotation().getRadians() ) < POSE_ERROR_THRESH;
    }

}