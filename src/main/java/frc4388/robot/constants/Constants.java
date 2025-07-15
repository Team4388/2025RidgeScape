/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc4388.utility.compute.Trim;
import frc4388.utility.status.CanDevice;
import frc4388.utility.structs.Gains;
import frc4388.utility.structs.LEDPatterns;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String CANBUS_NAME = "rio";

    public static final class LiDARConstants {
        public static final int REEF_LIDAR_DIO_CHANNEL = 7;
        public static final int REVERSE_LIDAR_DIO_CHANNEL = 4;

        public static final int HUMAN_PLAYER_STATION_DISTANCE = 40;

        public static final int LIDAR_DETECT_DISTANCE = 100; // Min distance to detect pole
        public static final int LIDAR_MICROS_TO_CM = 10;
        public static final int SECONDS_TO_MICROS = 1000000;
    }

    public static final class AutoConstants {
        // public static final Gains XY_GAINS = new Gains(5,0.6,0.0);
        public static final Gains XY_GAINS = new Gains(8,0,0.0);
        // public static final ConfigurableDouble P_XY_GAINS = new ConfigurableDouble("P_XY_GAINS", XY_GAINS.kP);
        // public static final ConfigurableDouble I_XY_GAINS = new ConfigurableDouble("I_XY_GAINS", XY_GAINS.kI);
        // public static final ConfigurableDouble D_XY_GAINS = new ConfigurableDouble("D_XY_GAINS", XY_GAINS.kD);
       // public static final Gains XY_GAINS = new Gains(3,0.3,0.0);

        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.007);
        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.0);

        public static final Trim X_OFFSET_TRIM =        new Trim("X Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE,0.5, 0);
        // public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 1.5);
        public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 0);
        public static final Trim ELEVATOR_OFFSET_TRIM = new Trim("Elevator Offset Trim", -ElevatorConstants.MAX_POSITION_ELEVATOR, ElevatorConstants.MAX_POSITION_ELEVATOR, 1, 0);
        public static final Trim ARM_OFFSET_TRIM =      new Trim("ARM Offset Trim",      -ElevatorConstants.COMPLETLY_TOP_ENDEFFECTOR, ElevatorConstants.COMPLETLY_TOP_ENDEFFECTOR, 1, 0);
                
        public static final double XY_TOLERANCE = 0.07; // Meters
        public static final double ROT_TOLERANCE = 5; // Degrees

        public static final double MIN_XY_PID_OUTPUT = 0.0;
        public static final double MIN_ROT_PID_OUTPUT = 0.0;

        public static final double VELOCITY_THRESHHOLD = 0.01;
                
        // X is tangent to reef side
        // Y is normal to reef side
        public static final double X_SCORING_POSITION_OFFSET = Units.inchesToMeters(6.5+1); // This is from the field
        public static final double Y_SCORING_POSITION_OFFSET = Units.inchesToMeters(16+1);
        public static final double HALF_ROBOT_SIZE = Units.inchesToMeters(18);

        public static final double L4_DISTANCE_PREP = HALF_ROBOT_SIZE + Units.inchesToMeters(15);
        public static final double L4_DISTANCE_SCORE = L4_DISTANCE_PREP;
        // public static final double L4_DISTANCE_SCORE = HALF_ROBOT_SIZE + Units.inchesToMeters(4.5);
        
        public static final double L3_DISTANCE_PREP = HALF_ROBOT_SIZE + Units.inchesToMeters(15);
        public static final double L3_DISTANCE_SCORE = HALF_ROBOT_SIZE + Units.inchesToMeters(5+1);
        
        public static final double L2_PREP_DISTANCE = HALF_ROBOT_SIZE + Units.inchesToMeters(6);
        public static final double L2_SCORE_DISTANCE = HALF_ROBOT_SIZE + Units.inchesToMeters(0.5-2);

        public static final double ALGAE_REMOVAL_DISTANCE = HALF_ROBOT_SIZE;
        
        // public static final double L4_DISTANCE_PREP = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(15);
        // public static final double L4_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(5.5);
        // // public static final double L4_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(4.5);
        
        // public static final double L3_DISTANCE_PREP = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(15);
        // public static final double L3_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(5+1);
        
        // public static final double L2_PREP_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(6);
        // public static final double L2_SCORE_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(0.5);

        // public static final double ALGAE_REMOVAL_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(2);

        public static final int L4_DRIVE_TIME = 250; //Milliseconds
        // public static final int L3_DRIVE_TIME = 500;
        public static final int L2_DRIVE_TIME = 250; //Milliseconds
        public static final int ALGAE_DRIVE_TIME = 500;
        public static final double STOP_VELOCITY = 0.0;
    }

    public static final class VisionConstants { 
        public static final String LEFT_CAMERA_NAME = "CAMERA_LEFT";
        public static final String RIGHT_CAMERA_NAME = "CAMERA_RIGHT";

        public static final Transform3d LEFT_CAMERA_POS = new Transform3d(new Translation3d(Units.inchesToMeters(4.547), Units.inchesToMeters(8.031), Units.inchesToMeters(8.858)), new Rotation3d(0,0.0,0.0));
        public static final Transform3d RIGHT_CAMERA_POS = new Transform3d(new Translation3d(Units.inchesToMeters(4.547), -Units.inchesToMeters(8.031), Units.inchesToMeters(8.858)), new Rotation3d(0,0.0,0.0));
        
        public static final double MIN_ESTIMATION_DISTANCE = 2; // Meters

        // Photonvision thing
        // The standard deviations of our vision estimated poses, which affect correction rate
        // X, Y, Theta
        // https://www.chiefdelphi.com/t/photonvision-finding-standard-deviations-for-swervedriveposeestimator/467802/2
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
    }

    public static final class FieldConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Test april tag field layout
        // public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(
        //     Arrays.asList(new AprilTag[] {
        //         new AprilTag(1, new Pose3d(
        //             new Translation3d(0.,0.,0.26035), new Rotation3d(0.,0.,0.)
        //         )),
        //     }), 100, 100);

    }

    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 9;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;

        public static final LEDPatterns WAITING_PATTERN = LEDPatterns.SOLID_RED;
        public static final LEDPatterns DOWN_PATTERN = LEDPatterns.SOLID_YELLOW;
        public static final LEDPatterns READY_PATTERN = LEDPatterns.SOLID_GREEN_DARK;
        public static final LEDPatterns SCORING_PATTERN = LEDPatterns.RAINBOW_RAINBOW;

        public static final LEDPatterns RED_PATTERN = LEDPatterns.LAVA_WAVES;
        public static final LEDPatterns BLUE_PATTERN = LEDPatterns.OCEAN_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
        public static final int BUTTONBOX_ID = 2;
        public static final int XBOX_PROGRAMMER_ID = 3;
        public static final double LEFT_AXIS_DEADBAND = 0.1;

    }

    public static final class ElevatorConstants {
        public static final CanDevice ENDEFFECTOR_ID = new CanDevice("Endeffector", 15);
        public static final CanDevice ELEVATOR_ID = new CanDevice("Elevator", 16);

        public static final double SAFETY_ENDEFFECTOR_MAX_TORQUE = 75;
        public static final double SAFETY_ENDEFFECTOR_MIN_VELOCITY = 20;

        // public static final int BASIN_LIMIT_SWITCH = 8; // TODO: FIND
        
        public static final int BASIN_LIMIT_SWITCH = 8; // TODO: FIND
        public static final int ENDEFFECTOR_LIMIT_SWITCH = 9; // TODO: FIND
        public static final int INTAKE_LIMIT_SWITCH = 6; // TODO: FIND

        
        public static final double GEAR_RATIO_ELEVATOR = -9.0;
        //Max for elevator = 50%
        
        public static final double GROUND_POSITION_ELEVATOR = 0 * GEAR_RATIO_ELEVATOR;
        public static final double WAITING_POSITION_ELEVATOR = -9.5; // TODO: find 2-4 in off the pipe
        public static final double HOVERING_POSITION_ELEVATOR = -7.5; // TODO: find 2-4 in off the pipe
        public static final double WAITING_POSITION_BEAM_BREAK_ELEVATOR = -5; // TODO: find on the pipe
        public static final double SCORING_THREE_ELEVATOR = -9.25;
        public static final double DEALGAE_L2_ELEVATOR = -4;
        public static final double DEALGAE_L3_ELEVATOR = -26.5;
        public static final double MAX_POSITION_ELEVATOR = 4.5 * GEAR_RATIO_ELEVATOR; // TODO: find MAX position
        
        public static final double GEAR_RATIO_ENDEFECTOR = -100.0;
        public static final double ENDEFECTOR_DRIVE_SLOW = -0.08;
        //Max for endefector = 60%
        public static final double L2_SCORE_ELEVATOR = -5;
        public static final double L2_LEAVE_ELEVATOR = -11;

        public static final double L2_SCORE_ENDEFFECTOR = -19;

        public static final double COMPLETLY_DOWN_ENDEFFECTOR = 0 * GEAR_RATIO_ENDEFECTOR;
        public static final double DEALGAE_L2_ENDEFFECTOR = 0.22 * GEAR_RATIO_ENDEFECTOR;
        public static final double COMPLETLY_MIDDLE_ENDEFFECTOR = 0.25 * GEAR_RATIO_ENDEFECTOR;
        public static final double PRIMED_THREE_ENDEFFECTOR = 0.4 * GEAR_RATIO_ENDEFECTOR;
        public static final double SCORING_FOUR_ENDEFFECTOR = 0.3 * GEAR_RATIO_ENDEFECTOR;
        public static final double PRIMED_FOUR_ENDEFFECTOR = 0.44 * GEAR_RATIO_ENDEFECTOR;
        public static final double COMPLETLY_TOP_ENDEFFECTOR = 0.5 * GEAR_RATIO_ENDEFECTOR;

        public static final Slot0Configs ELEVATOR_PID = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0);
        
        public static final Slot0Configs ENDEFFECTOR_PID = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0);
    }

    // Logging constants
    public static class SimConstants {
        public static final Mode simMode = Mode.SIM;
        public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

        public static enum Mode {
            /** Running on a real robot. */
            REAL,

            /** Running a physics simulator. */
            SIM,

            /** Replaying from a log file. */
            REPLAY
        }
    }
}
