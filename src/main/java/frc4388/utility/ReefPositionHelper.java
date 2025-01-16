package frc4388.utility;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc4388.robot.Constants.FieldConstants;

public class ReefPositionHelper {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static Pose2d[] RED_TAGS = {
        FieldConstants.kTagLayout.getTagPose(6).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(7).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(8).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(9).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(10).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(11).get().toPose2d()
    };

    public static Pose2d[] BLUE_TAGS = {
        FieldConstants.kTagLayout.getTagPose(17).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(18).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(19).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(20).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(21).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(22).get().toPose2d()
    };

    public static double distanceTo(Pose2d first, Pose2d second){
        return Math.sqrt(  Math.pow(first.getX()  -  second.getX(),2) + Math.pow(first.getY()  -  second.getY(),2));
    }


    /*
    * Function to loop through a list of tag locations to figure out closest one
    */
    public static Pose2d getNearestTag(Pose2d[] locations, Pose2d position){
        if(locations.length <= 0) return new Pose2d();

        Pose2d minPos = locations[0];
        double minDistance = distanceTo(position,minPos);

        for(int i = 1; i < locations.length; i++){
            double dist = distanceTo(locations[i],minPos);
            if(dist < minDistance){
                minPos = locations[i];
                minDistance = dist;
            }
        }

        return minPos;
    }

    /*
     * Function to find closest tag location based on side
     */
    public static Pose2d getNearestTag(Pose2d position) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (!ally.isPresent()) 
            return new Pose2d();
        if (ally.get() == Alliance.Red) 
            return getNearestTag(RED_TAGS, position);
        if (ally.get() == Alliance.Blue) 
            return getNearestTag(BLUE_TAGS, position);   
        return new Pose2d();
    }

    public static Pose2d getNearestPosition(Pose2d position, Side side, double xtrim) {
        return getNearestTag(horisontalOffset(position, side == Side.LEFT ? -(xtrim+FieldConstants.HORISONTAL_SCORING_POSITION_OFFSET) : (xtrim+FieldConstants.HORISONTAL_SCORING_POSITION_OFFSET))); 
    }


    public static Pose2d horisontalOffset(Pose2d oldPose, double xoffset){
        Translation2d oldTranslation = oldPose.getTranslation();
        Rotation2d rot = oldPose.getRotation();
        return new Pose2d(new Translation2d(
            oldTranslation.getX() + rot.getCos() * xoffset,
            oldTranslation.getY() + rot.getSin() * xoffset
        ), oldPose.getRotation());
    }
}
