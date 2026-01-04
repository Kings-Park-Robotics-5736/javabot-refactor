package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;



public final class Field {

    public static final double WIDTH = Units.inchesToMeters(317.000); 
    public static final double LENGTH = Units.inchesToMeters(690.876);

    public final class ScoringPositions {
        public static final Pose2d kBlueScoringPosition = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
        public static final Pose2d kRedScoringPosition = new Pose2d(new Translation2d(0, 5.55), Rotation2d.fromDegrees(0));
    
        
    };

    public static final int[] BLUE_TAG_IDS = {17, 18, 19, 20, 21, 22};
    public static final int[] RED_TAG_IDS = {6, 7, 8, 9, 10, 11};

    public static boolean IsRobotOnBlueSide(Pose2d pose){
        return pose.getX() < Field.LENGTH / 2;
    }
    
}