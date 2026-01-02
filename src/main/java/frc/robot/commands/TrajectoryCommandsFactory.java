package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.DriveToCoordinate;
import frc.robot.commands.drive.PathPlanFromDynamicStartCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.MathUtils;

public class TrajectoryCommandsFactory {

    private static PathConstraints constraints = new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);


    /*****************************
     * Helpers
     *****************************/
    public static PathPlannerPath getPathFromFile(String pathName) {
        PathPlannerPath path = null;
        try{
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e){
            System.out.println("Error loading path " + pathName);
        }
        return path;
    }

  

    /*******************************
     * Commands
     *******************************/

    /**
     * Generate a PathPlanner pathfind command to a specific pose
     * @param robotDrive
     * @param endPos
     * @return
     */
    public static Command generatePPPathFindToPose(DriveSubsystem robotDrive, Pose2d endPos) {

        return  AutoBuilder.pathfindToPose(
            endPos,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );
    }


    /**
     * Generate a PathPlanner pathfind command to a specific path (from file) from a dynamic start
     * @param robotDrive
     * @param pathName
     * @return
     */
    public static Command generatePPPathFindToPath(DriveSubsystem robotDrive, String pathName) {

        PathPlannerPath path = getPathFromFile(pathName);

        if( path != null){
            Pose2d endPose = new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
            path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
            path.getGoalEndState().rotation());

            return Commands.runOnce(()->System.out.println("Running Auto " + pathName + "with end pose " + endPose.toString())).andThen(AutoBuilder.pathfindThenFollowPath(path, constraints));
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    /**
     * Generate a PathPlanner pathfind command to a specific path (from file) from a dynamic start then align to end pose with extra logic
     * @param robotDrive
     * @param pathName
     * @return
     */
    public static Command generatePPPathFindToPathThenAlign(DriveSubsystem robotDrive, String pathName) {

        PathPlannerPath path = getPathFromFile(pathName);
      

        if( path != null){
            Pose2d endPose = new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
            path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
            path.getGoalEndState().rotation());

            return (Commands.runOnce(()->System.out.println("Running PP Path Find W/ Align to path " + pathName)) 
                    .andThen(
                        AutoBuilder.pathfindThenFollowPath(path, constraints))
                    .andThen(new PathPlanFromDynamicStartCommand(
                        () -> robotDrive.getPose(),
                        robotDrive,
                        endPose,
                        new ArrayList<PathPoint>(),
                        true
                    ))
                    .andThen(Commands.runOnce(()->System.out.println("Done Running PP Path Find W/ Align to path " + pathName)))).withName("PP Path Find W/ Align to path " + pathName);
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    /**
     * Use the manual drive to coordinate routine to drive to the end coordinate of a path (dont use the path)
     * @param robotDrive
     * @param pathName
     * @return
     */
    public static Command generatePPTrajectoryOnTheFlyFromPath(DriveSubsystem robotDrive, String pathName) {

        PathPlannerPath path = getPathFromFile(pathName);
      

        if( path != null){
            Pose2d endPose = new Pose2d(path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getX(), 
            path.getWaypoints().get(path.getWaypoints().size() - 1).anchor().getY(),
            path.getGoalEndState().rotation());

            return (Commands.runOnce(()->System.out.println("Running PP Path FLY Find W/ Align to path " + pathName)) 
                .andThen(new DriveToCoordinate(robotDrive, endPose))        
            
           
                    .andThen(Commands.runOnce(()->System.out.println("Done Running PP Path FLY Find W/ Align to path " + pathName)))).withName("PP On the fly" + pathName);
        }
        return Commands.run(() -> {
            System.out.println("Error loading path");
        });
    }

    /**
     * This method is used by PathPlanFromDynamicStartCommand to create a path, should not be used directly
     */
    public static Command generatePPPathToPose( List<Waypoint> waypoints, Rotation2d endRotation ) {

        PathConstraints constraints = new PathConstraints(4.0, 4.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      
        // Create the path using the waypoints created above
       
        return generatePPPathToPose(waypoints, endRotation, constraints);
    }

    public static Command generatePPPathToPose( List<Waypoint> waypoints, Rotation2d endRotation, PathConstraints constraints ) {

      
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, endRotation) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return AutoBuilder.followPath(path); 
    }
   
    
}
