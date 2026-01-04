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

public class LimelightVisionSubsystem extends SubsystemBase{


    public enum WhitelistMode {
        BLUE_TAGS(Field.BLUE_TAG_IDS),
        RED_TAGS(Field.RED_TAG_IDS);

        private int[] ids;

        private WhitelistMode(int... ids){
            this.ids = ids;
        }

        public int[] getIds() {
            return this.ids;
        }
    }

    
    private Limelight[] m_limelights;
    private WhitelistMode[] currentWhitelistModes = null;
    private int maxTagCount;
    private final DriveSubsystem m_driveSubsystem;
    private int imuMode = 1;
    private boolean m_isRobotDisabled = true;
    

    public LimelightVisionSubsystem(DriveSubsystem driveSubsystem, Limelight ... limelights) {
        m_limelights = limelights;
        m_driveSubsystem = driveSubsystem;

        setIMUMode(imuMode);
    }

   
    

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public void setIMUMode(int mode) {
        this.imuMode = mode;
        for (Limelight camera : m_limelights) {
            camera.SetIMUMode(mode);
        }
    }

    public void SetRobotDisabled(boolean isRobotDisabled) {
        m_isRobotDisabled = isRobotDisabled;

    }

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

    public void setWhitelistMode(WhitelistMode... modes) {
        int totalLength = 0;

        for (WhitelistMode mode : modes) {
            totalLength += mode.getIds().length;
        }
    
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


    private void setTagWhitelist(int... ids) {
        for (Limelight camera : m_limelights) {
            camera.SetFiducialIDFiltersOverride(ids);
        }
    } 

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

    private boolean robotIsOnBlueSide() {
        return Field.IsRobotOnBlueSide(m_driveSubsystem.getPose());
    }



    private void updateWhitelistMode() {
        if (m_isRobotDisabled) { // whitelist alliance tags during disabled loop
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                setWhitelistMode(WhitelistMode.BLUE_TAGS);
            } else {
                setWhitelistMode(WhitelistMode.RED_TAGS);
            }
        } else {
            //only send the update to the limelight if we changed
            if (robotIsOnBlueSide() && isCurrentlyWhitelisted(WhitelistMode.RED_TAGS)) {
                setWhitelistMode(WhitelistMode.BLUE_TAGS);
            }
            if (!robotIsOnBlueSide() && isCurrentlyWhitelisted(WhitelistMode.BLUE_TAGS)) {
                setWhitelistMode(WhitelistMode.RED_TAGS);
            }
        }
    }


     @Override
    public void periodic() {
        this.maxTagCount = 0;

        updateWhitelistMode();

        for (Limelight camera : m_limelights) {
            camera.SetRobotOrientation(m_driveSubsystem.getPose().getRotation().getDegrees());

            //from CTRE docs:
            double omegaRps = Units.radiansToRotations(m_driveSubsystem.getState().Speeds.omegaRadiansPerSecond);

            PoseEstimate poseEstimate = camera.GetMegatagPoseEstimate();

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

        // SmartDashboard.putString("Vision/Whitelist Mode", getWhitelistModes().toString());
        SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
    }
}
