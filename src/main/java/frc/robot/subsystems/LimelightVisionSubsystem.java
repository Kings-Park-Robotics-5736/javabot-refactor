package frc.robot.subsystems;   
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.field.Field;
import frc.robot.subsystems.phoenixDrive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.vision.Limelight;

/**
 * Subsystem for managing multiple Limelight cameras for vision-based pose estimation.
 * 
 * This subsystem coordinates multiple Limelight cameras to provide robot localization
 * using AprilTag detection. It handles:
 * - AprilTag ID filtering based on alliance and field position
 * - Pose estimation using MegaTag algorithms (MT1/MT2)
 * - Integration with the drivetrain for vision measurements
 * - Dynamic standard deviation adjustment based on robot state
 * - SmartDashboard telemetry for vision data
 */
public class LimelightVisionSubsystem extends SubsystemBase{

    /**
     * Enum defining AprilTag whitelist modes for filtering visible tags.
     * 
     * The whitelist restricts which AprilTags the Limelight will process,
     * helping to avoid false positives from tags on the opposite alliance.
     */
    public enum WhitelistMode {
        /** Blue alliance AprilTags */
        BLUE_TAGS(Field.BLUE_TAG_IDS),
        /** Red alliance AprilTags */
        RED_TAGS(Field.RED_TAG_IDS);

        private int[] ids;

        private WhitelistMode(int... ids){
            this.ids = ids;
        }

        /**
         * Get the array of AprilTag IDs for this whitelist mode.
         * 
         * @return Array of AprilTag IDs
         */
        public int[] getIds() {
            return this.ids;
        }
    }

    /** Array of Limelight cameras managed by this subsystem */
    private Limelight[] m_limelights;
    
    /** Currently active whitelist modes across all cameras */
    private WhitelistMode[] currentWhitelistModes = null;
    
    /** Maximum number of AprilTags seen by any camera in the current cycle */
    private int maxTagCount;
    
    /** Reference to the drive subsystem for pose integration */
    private final DriveSubsystem m_driveSubsystem;
    
    /** IMU mode setting for the Limelight cameras (0=disabled, 1=enabled) */
    private int imuMode = 1;
    
    /** Whether the robot is currently disabled */
    private boolean m_isRobotDisabled = true;
    

    /**
     * Constructs a LimelightVisionSubsystem with the specified cameras.
     * 
     * @param driveSubsystem The drive subsystem for pose estimation integration
     * @param limelights Variable number of Limelight cameras to manage
     */
    public LimelightVisionSubsystem(DriveSubsystem driveSubsystem, Limelight ... limelights) {
        m_limelights = limelights;
        m_driveSubsystem = driveSubsystem;

        setIMUMode(imuMode);
    }

    /**
     * Gets the maximum number of AprilTags seen by any camera in the current periodic cycle.
     * 
     * This can be used to assess the quality of vision measurements - more tags
     * typically means more reliable localization.
     * 
     * @return Maximum tag count across all cameras
     */
    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    /**
     * Sets the IMU mode for all Limelight cameras.
     * 
     * @param mode IMU mode (0=disabled, 1=enabled). When enabled, the Limelight
     *             uses IMU data to improve pose estimation accuracy.
     */
    public void setIMUMode(int mode) {
        this.imuMode = mode;
        for (Limelight camera : m_limelights) {
            camera.SetIMUMode(mode);
        }
    }

    /**
     * Sets whether the robot is currently disabled.
     * 
     * This affects whitelist behavior and standard deviation calculations.
     * When disabled, the subsystem uses alliance-based whitelisting and
     * different vision measurement confidence values.
     * 
     * @param isRobotDisabled True if robot is disabled, false otherwise
     */
    public void SetRobotDisabled(boolean isRobotDisabled) {
        m_isRobotDisabled = isRobotDisabled;

    }

    /**
     * Gets the appropriate standard deviations for vision measurements based on
     * the Limelight's MegaTag mode and robot state.
     * 
     * Standard deviations represent the confidence in the vision measurement:
     * - MegaTag1 (single tag): Uses different values for stationary vs. moving robot
     * - MegaTag2 (multi-tag): Uses consistent values (more reliable)
     * 
     * @param ll The Limelight camera to get standard deviations for
     * @return 3D vector of standard deviations [x, y, theta] in meters and radians
     */
    private Vector<N3> getStdDevs(Limelight ll){
        if (ll.GetMegatagMode() == Limelight.MegaTagMode.MEGATAG1) {
            if(!m_isRobotDisabled){
                return VisionConstants.MT1_STDEVS_STATIONARY;
            }else{
                return VisionConstants.MT1_STDEVS;
            }
        } else {
            return VisionConstants.MT2_STDEVS;
        }
    }

    /**
     * Sets the AprilTag whitelist mode(s) for all cameras.
     * 
     * This method accepts one or more whitelist modes and combines their tag IDs
     * to create a unified whitelist. For example, passing both BLUE_TAGS and RED_TAGS
     * would allow all field tags to be visible.
     * 
     * @param modes One or more WhitelistMode values to enable
     */
    public void setWhitelistMode(WhitelistMode... modes) {
        // Calculate total length of combined tag ID array
        int totalLength = 0;

        for (WhitelistMode mode : modes) {
            totalLength += mode.getIds().length;
        }
    
        // Combine all tag IDs from the specified modes
        int[] combined = new int[totalLength];
        int index = 0;
        for (WhitelistMode mode : modes) {
            for (int id : mode.getIds()) {
                combined[index++] = id;
            }
        }

        currentWhitelistModes = modes;

        setTagWhitelist(combined); 
    }

    /**
     * Applies the specified AprilTag IDs as a whitelist filter to all cameras.
     * 
     * @param ids Variable number of AprilTag IDs to whitelist
     */
    private void setTagWhitelist(int... ids) {
        for (Limelight camera : m_limelights) {
            camera.SetFiducialIDFiltersOverride(ids);
        }
    } 

    /**
     * Checks if the specified whitelist mode(s) are currently active.
     * 
     * Returns true only if ALL specified modes are currently active.
     * 
     * @param modes One or more WhitelistMode values to check
     * @return True if all specified modes are active, false otherwise
     */
    public boolean isCurrentlyWhitelisted(WhitelistMode... modes) {
        if (currentWhitelistModes != null) {
            int count = 0;
            for (WhitelistMode mode : modes) {
                for (WhitelistMode m : this.currentWhitelistModes) {
                    if (m.equals(mode)) {
                        count++;
                    }
                }
            }
            return count == modes.length;
        }
        return false;
    }

    /**
     * Determines if the robot is currently on the blue alliance side of the field.
     * 
     * This is used for dynamic whitelist switching during enabled mode to prevent
     * the robot from using incorrect tags when crossing the field centerline.
     * 
     * @return True if robot is on blue side, false if on red side
     */
    private boolean robotIsOnBlueSide() {
        return Field.IsRobotOnBlueSide(m_driveSubsystem.getPose());
    }

    public WhitelistMode[] getCurrentWhitelistModes() {
        return this.currentWhitelistModes;
    }

    /**
     * Updates the AprilTag whitelist based on robot state and field position.
     * 
     * Behavior:
     * - When disabled: Uses alliance color from DriverStation
     * - When enabled: Dynamically switches based on field position
     *   - Blue side of field: Uses blue alliance tags
     *   - Red side of field: Uses red alliance tags
     * 
     * This prevents false localization from using the wrong alliance's tags
     * when the robot crosses the field centerline during autonomous or teleop.
     */
    private void updateWhitelistMode() {
        if (m_isRobotDisabled) {
            // Whitelist alliance tags during disabled - based on DriverStation alliance
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                setWhitelistMode(WhitelistMode.BLUE_TAGS);
            } else {
                setWhitelistMode(WhitelistMode.RED_TAGS);
            }
        } else {
            // Only send the update to the limelight if the whitelist changed
            if (robotIsOnBlueSide() && isCurrentlyWhitelisted(WhitelistMode.RED_TAGS)) {
                setWhitelistMode(WhitelistMode.BLUE_TAGS);
            }
            if (!robotIsOnBlueSide() && isCurrentlyWhitelisted(WhitelistMode.BLUE_TAGS)) {
                setWhitelistMode(WhitelistMode.RED_TAGS);
            }
        }
    }

    /**
     * Periodic method called every robot loop (~20ms).
     * 
     * This method:
     * 1. Updates the AprilTag whitelist based on robot state
     * 2. Updates each camera with current robot orientation
     * 3. Retrieves pose estimates from each camera
     * 4. Adds valid vision measurements to the drive subsystem's pose estimator
     * 5. Publishes telemetry data to SmartDashboard
     * 
     * Vision measurements are only added when:
     * - A pose estimate is available
     * - At least one AprilTag is detected
     * - Robot angular velocity is below 2.0 rotations/second (reduces motion blur)
     */
    @Override
    public void periodic() {
        // Reset max tag count for this cycle
        this.maxTagCount = 0;

        // Update tag whitelist based on current state/position
        updateWhitelistMode();

        // Process each camera
        for (Limelight camera : m_limelights) {
            // Update camera with current robot orientation for MegaTag
            camera.SetRobotOrientation(m_driveSubsystem.getPose().getRotation().getDegrees());

            // Get angular velocity in rotations per second (from CTRE docs)
            double omegaRps = Units.radiansToRotations(m_driveSubsystem.getState().Speeds.omegaRadiansPerSecond);

            // Get the latest pose estimate from this camera
            PoseEstimate poseEstimate = camera.GetMegatagPoseEstimate();

            // Only use vision measurement if valid and robot isn't spinning too fast
            if (poseEstimate != null && poseEstimate.tagCount > 0 && omegaRps < 2.0) {
                m_driveSubsystem.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, getStdDevs(camera));
                SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
                SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
                maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
            }
            else {
                SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
                SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", 0);
            }

            SmartDashboard.putString("Vision/" + camera.getName() + "Megatag Mode/", camera.GetMegatagMode().toString());

            
        }

        // Publish global vision telemetry
        SmartDashboard.putString("Vision/Whitelist Mode", getCurrentWhitelistModes().toString());
        SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
    }
}
