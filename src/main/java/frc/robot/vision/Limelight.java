package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.utils.LimelightHelpers;

/**
 * Limelight camera class
 */
public class Limelight {
    private String name;
    private Boolean m_supportsIMU;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    private MegaTagMode megaTagMode = MegaTagMode.MEGATAG1;


    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        private int value;

        private LEDMode(int mode) {
            this.value = mode;
        }
    }

    public enum CamMode {
        VISION(0),
        DRIVER(1);

        private int value;

        private CamMode(int mode) {
            this.value = mode;
        }
    }

    /**
     * Camera sends data to network table, get table and values when creating
     * instance of Limelight
     */
    public Limelight(String tableName, Boolean supportsIMU) {
        name = tableName;
        m_supportsIMU = supportsIMU;
    }

    public String getName(){
        return name;
    }

    public MegaTagMode GetMegatagMode(){
        return megaTagMode;
    }

    public void SetMegatagMode(MegaTagMode mode){
        megaTagMode = mode;
    }

    public void SetRobotOrientation(double degrees){
        LimelightHelpers.SetRobotOrientation(name, degrees, 0, 0, 0, 0, 0);

    }


    public LimelightHelpers.PoseEstimate GetBotPoseMT2(){
        return  LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }     

    public LimelightHelpers.PoseEstimate GetBotPoseMT1(){
        return  LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }
    

    public LimelightHelpers.PoseEstimate GetMegatagPoseEstimate(){
        if(megaTagMode == MegaTagMode.MEGATAG2){
            return GetBotPoseMT2();
        }else{
            return GetBotPoseMT1();
        }
    }
    
  
    public void SetIMUMode(int mode){
        if(m_supportsIMU){
            LimelightHelpers.SetIMUMode(name, mode);
        }
    }



    /**
     * Set LED mode
     * 
     * @param mode LEDMode(0-4)
     */
    public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case PIPELINE:
                LimelightHelpers.setLEDMode_PipelineControl(name);
                break;
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(name);
                break;
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(name);
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(name);
                break;
        }
    }

    /**
     * Set LED mode to ON
     */
    public void setLEDOn() {
        LimelightHelpers.setLEDMode_ForceOn(name);
    }

    public Command TurnOnLEDsFor3Sec(){
        return Commands.runOnce(()->setLEDOn()).andThen(Commands.waitSeconds(3).andThen(Commands.runOnce(()->setLEDOff())));
    }

    /**
     * Set LED mode to OFF
     */
    public void setLEDOff() {
        LimelightHelpers.setLEDMode_ForceOff(name);
    }

    /**
     * Set LED mode to BLINK
     */
    public void setLEDBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(name);
    }

    /**
     * Set camera mode
     * 
     * @param mode CamMode(0-1)
     */
    public void setCamMode(CamMode mode) {
        // LimelightHelpers doesn't have a direct camera mode setter
        // Using NetworkTables directly for this functionality
        LimelightHelpers.getLimelightNTTable(name).getEntry("camMode").setNumber(mode.value);
    }

    public void SetFiducialIDFiltersOverride( int[] validIDs){
        LimelightHelpers.SetFiducialIDFiltersOverride(name, validIDs);
    }

    /**
     * Preset operation mode - LED set to OFF, camera set to DRIVER
     */
    public void setModeDriver() {
        this.setLEDMode(LEDMode.OFF);
        this.setCamMode(CamMode.DRIVER);
    }

    /**
     * Preset operation mode - LED set to ON, camera set to VISION
     */
    public void setModeVision() {
        this.setLEDMode(LEDMode.ON);
        this.setCamMode(CamMode.VISION);
    }

    public int getActualPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(name);
    }

    public boolean getIsPipelineReflective() {
        return getActualPipeline() == 1;
    }

    public boolean getIsPipelineAprilTag() {
        return getActualPipeline() == 0;
    }

    public void setPipeline(int pipleline) {
        LimelightHelpers.setPipelineIndex(name, pipleline);
    }

    public void setAprilTagPipeline() {
        this.setPipeline(0);
    }

    public void setReflectivePipeline() {
        this.setPipeline(1);
    }

    /**
     * Get horizontal offset to target
     * 
     * @return -29.8 - 29.8 degrees
     */
    public double getTargetOffsetX() {
        return LimelightHelpers.getTX(name);
    }

    public long getLastOffsetXChange() {
        return LimelightHelpers.getLimelightNTTable(name).getEntry("tx").getLastChange();
    }

    /**
     * Get vertical offset to target
     * 
     * @return -24.85 - 24.85 degrees
     */
    public double getTargetOffsetY() {
        return LimelightHelpers.getTY(name);
    }

    /**
     * Check for a detected target
     * 
     * @return boolean - true if target is found else false
     */
    public boolean checkValidTarget() {
        return LimelightHelpers.getTV(name);
    }

    /**
     * Get target area
     * 
     * @return 0% - 100% of image
     */
    public double getTargetArea() {
        return LimelightHelpers.getTA(name);
    }

    /**
     * Get ID of primary in-view AprilTag
     * 
     * @return double
     */
    public double getTargetID() {
        return LimelightHelpers.getFiducialID(name);
    }

    public String poseToString(double[] pose) {
        String ret = "";
        for (double p : pose) {
            ret += p + ", ";
        }
        return ret;
    }

    public Pose2d AsPose2d(double[] pose) {
        return new Pose2d(pose[0], pose[1], new Rotation2d(Units.degreesToRadians(pose[5])));
    }

    /**
     * Get botpose
     * 
     * @return double[]
     */
    public double[] getBotPose() {
        return LimelightHelpers.getBotPose(name);
    }

    /**
     * Get botpose for blue field side
     * 
     * @return double[]
     */
    public double[] getBotPoseBlue() {
        return LimelightHelpers.getBotPose_wpiBlue(name);
    }

    /**
     * Get botpose for red field side
     * 
     * @return double[]
     */
    public double[] getBotPoseRed() {
        return LimelightHelpers.getBotPose_wpiRed(name);
    }

    /**
     * Get camera 3D transform in the coordinate system of the primary in-view
     * AprilTag
     * 
     * @return double[]
     */
    public double[] getCameraPoseTargetSpace() {
        return LimelightHelpers.getCameraPose_TargetSpace(name);
    }

    /**
     * Get AprilTag 3D transform in the coordinate system of the camera
     * 
     * @return double[]
     */
    public double[] getTargetPoseCameraSpace() {
        return LimelightHelpers.getTargetPose_CameraSpace(name);
    }

    /**
     * Get AprilTag 3D transform in the coordinate system of the robot
     * 
     * @return double[]
     */
    public double[] getTargetPoseBotSpace() {
        return LimelightHelpers.getTargetPose_RobotSpace(name);
    }

    /**
     * Get robot 3D transform in the coordinate system of the AprilTag
     * 
     * @return double[]
     */
    public double[] getBotPoseTargetSpace() {
        return LimelightHelpers.getBotPose_TargetSpace(name);
    }


    /**
     * Get distance to AprilTag
     * 
     * @return double
     */
    public double getTargetDistance() {
        // vertical offset / tan(a1+a2)
        return 0.0;
    }

    /**
     * Command - toggle LED ON/OFF
     * 
     * @return Command
     *
     *         public Command ToggleLEDCommand() {
     *         return new FunctionalCommand(
     *         () -> {},
     *         () -> this.setLEDOn(),
     *         (interrupted) -> this.setLEDOff(),
     *         () -> false, this);
     *         }
     */

    /**
     * Command - toggle limelight LED ON/OFF
     */
    public void toggleLED() {
        // Get current LED mode from NetworkTables since LimelightHelpers doesn't have a getter
        double currentMode = LimelightHelpers.getLimelightNTDouble(name, "ledMode");
        if (currentMode == LEDMode.ON.value) {
            this.setLEDOff();
        } else {
            this.setLEDOn();
        }
    }

}
