package frc4388.utility.compute;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.constants.DriveConstants;

// Class that holds weather the drivers sticks should be inverted
public class TimesNegativeOne {

    public static boolean XAxis = DriveConstants.INVERT_X;
    public static boolean YAxis = DriveConstants.INVERT_Y;
    public static boolean RotAxis = DriveConstants.INVERT_ROTATION;
    public static boolean isRed = false;
    public static Rotation2d ForwardOffset = Rotation2d.fromDegrees(DriveConstants.FORWARD_OFFSET);

    private static boolean isRed() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) return false;
        return (alliance.get() == Alliance.Red);
    }

    public static void update(){
        isRed = isRed();
        XAxis = DriveConstants.INVERT_X ^ isRed;
        YAxis = DriveConstants.INVERT_Y ^ isRed;
        RotAxis = DriveConstants.INVERT_ROTATION;
        ForwardOffset = Rotation2d.fromDegrees((DriveConstants.FORWARD_OFFSET + (isRed ? 0 : 0)));
        SmartDashboard.putBoolean("Is red alliance", isRed);
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
