package frc4388.utility;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc4388.robot.Constants.SwerveDriveConstants;

// Class that holds weather the drivers sticks should be inverted
public class TimesNegativeOne {

    public static boolean XAxis = SwerveDriveConstants.INVERT_X;
    public static boolean YAxis = SwerveDriveConstants.INVERT_Y;
    public static boolean RotAxis = SwerveDriveConstants.INVERT_ROTATION;
    public static boolean isRed = false;

    private static boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) return false;
        return (alliance.get() == Alliance.Red);
    }

    public static void update(){
        isRed = isRed();
        XAxis = SwerveDriveConstants.INVERT_X ^ isRed;
        YAxis = SwerveDriveConstants.INVERT_Y ^ isRed;
        RotAxis = SwerveDriveConstants.INVERT_ROTATION;
    }

    public static double invert(double num, boolean invert){
        return invert ? -num : num;
    }

    public static Translation2d invert(Translation2d stick, boolean invertXY){
        if(invertXY) return stick.times(-1);
        else return stick;
    }

    public static Translation2d invert(Translation2d stick, boolean invertX, boolean invertY){
        return new Translation2d(
            invert(stick.getX(), invertX), 
            invert(stick.getY(), invertY)
        );
    }
}
