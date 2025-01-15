/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc4388.utility.CanDevice;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;
import frc4388.utility.Trim;

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
    
    public static final class SwerveDriveConstants {

        public static final double MAX_ROT_SPEED        = 3.5;
        public static final double AUTO_MAX_ROT_SPEED = 1.5;
        public static final double MIN_ROT_SPEED        = 1.0;
        public static       double ROTATION_SPEED       = MAX_ROT_SPEED;
        public static       double PLAYBACK_ROTATION_SPEED = AUTO_MAX_ROT_SPEED;
        public static       double ROT_CORRECTION_SPEED = 10; // MIN_ROT_SPEED;

        public static final double CORRECTION_MIN = 10;
        public static final double CORRECTION_MAX = 50;
        
        public static final double SLOW_SPEED = 0.25;
        public static final double FAST_SPEED = 0.5;
        public static final double TURBO_SPEED = 1.0;
        
        public static final double[] GEARS = {SLOW_SPEED, FAST_SPEED, TURBO_SPEED};
        public static final int STARTING_GEAR = 0;
        // dimensions
        public static final double WIDTH = 18.5; // TODO: validate
        public static final double HEIGHT = 18.5; // TODO: validate
        public static final double HALF_WIDTH = WIDTH / 2.d;
        public static final double HALF_HEIGHT = HEIGHT / 2.d;

        // Mechanics
        private static final double COUPLE_RATIO = 3; //todo: find
        private static final double DRIVE_RATIO = 6.12;
        private static final double STEER_RATIO = (150/7);
        private static final Distance WHEEL_RADIUS = Inches.of(2);
        
        public static final double MAX_SPEED_MEETERS_PER_SEC = 6.22; // TODO: Validate

        public static final double MAX_SPEED_FEET_PER_SECOND = MAX_SPEED_MEETERS_PER_SEC * 3.28084;
        public static final double MAX_ANGULAR_SPEED_FEET_PER_SECOND = 2 * 2 * Math.PI;

        // public static final double FORWARD_OFFSET = 270; // 0, 90, 180, 270
        public static final double FORWARD_OFFSET = 0; // 0, 90, 180, 270

        public static final Trim POINTLESS_TRIM = new Trim("Pointless Trim", Double.MAX_VALUE, Double.MIN_VALUE, 1.d, 10);

        private static final class ModuleSpecificConstants {
            //Front Left
            // private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.36328125);
            private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.229736328125+.25);
            private static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
            private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;
            // private static final Distance FRONT_LEFT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance FRONT_LEFT_XPOS = Inches.of(-HALF_WIDTH);
            private static final Distance FRONT_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            
            //Front Right
            // private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(0.133056640625);
            private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(-0.371337890625+.5+.25);
            private static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
            private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;
            private static final Distance FRONT_RIGHT_XPOS = Inches.of(HALF_WIDTH);
            // private static final Distance FRONT_RIGHT_YPOS = Inches.of(-HALF_HEIGHT);
            private static final Distance FRONT_RIGHT_YPOS = Inches.of(HALF_HEIGHT);

            //Back Left
            // private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.47705078125 + 0.5);
            private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.3828125+.25+.5);
            private static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
            private static final boolean BACK_LEFT_ENCODER_INVERTED = false;
            private static final Distance BACK_LEFT_XPOS = Inches.of(-HALF_WIDTH);
            // private static final Distance BACK_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            private static final Distance BACK_LEFT_YPOS = Inches.of(-HALF_HEIGHT);
            
            //Back Right
            // private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(-0.355224609375 + 0.5);
            private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(-0.01904296875+.25);
            private static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
            private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;
            // private static final Distance BACK_RIGHT_XPOS = Inches.of(-HALF_WIDTH);
            private static final Distance BACK_RIGHT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance BACK_RIGHT_YPOS = Inches.of(-HALF_HEIGHT);
        }

        public static final class IDs {
            public static final CanDevice RIGHT_FRONT_WHEEL   = new CanDevice("RIGHT_FRONT_WHEEL", 4);
            public static final CanDevice RIGHT_FRONT_STEER   = new CanDevice("RIGHT_FRONT_STEER", 5);
            public static final CanDevice RIGHT_FRONT_ENCODER = new CanDevice("RIGHT_FRONT_ENCODER", 11);
            
            public static final CanDevice LEFT_FRONT_WHEEL    = new CanDevice("LEFT_FRONT_WHEEL", 2);
            public static final CanDevice LEFT_FRONT_STEER    = new CanDevice("LEFT_FRONT_STEER", 3);
            public static final CanDevice LEFT_FRONT_ENCODER  = new CanDevice("LEFT_FRONT_ENCODER", 10);
            
            public static final CanDevice LEFT_BACK_WHEEL     = new CanDevice("LEFT_BACK_WHEEL", 6);
            public static final CanDevice LEFT_BACK_STEER     = new CanDevice("LEFT_BACK_STEER", 7);
            public static final CanDevice LEFT_BACK_ENCODER   = new CanDevice("LEFT_BACK_ENCODER", 12);
            
            public static final CanDevice RIGHT_BACK_WHEEL    = new CanDevice("RIGHT_BACK_WHEEL", 8);  
            public static final CanDevice RIGHT_BACK_STEER    = new CanDevice("RIGHT_BACK_STEER", 9);
            public static final CanDevice RIGHT_BACK_ENCODER  = new CanDevice("RIGHT_BACK_ENCODER", 13);

            public static final CanDevice DRIVE_PIGEON        = new CanDevice("DRIVE_PIGEON", 14);
        }
    
        public static final class PIDConstants {
            public static final int SWERVE_SLOT_IDX = 0;
            public static final int SWERVE_PID_LOOP_IDX = 1;
            public static final Gains SWERVE_GAINS = new Gains(50, 0.0, 0.32, 0.0, 0, 0.0);

            public static final Slot0Configs CURRENT_SWERVE_ROT_GAINS = new Slot0Configs()
                .withKP(50).withKI(0).withKD(0.32)
                .withKS(0).withKV(0).withKA(0);
            
                public static final Slot0Configs TEST_SWERVE_ROT_GAINS = new Slot0Configs()
                .withKP(10).withKI(0).withKD(0.3)
                .withKS(0).withKV(0).withKA(0);

            public static final Gains TEST_SWERVE_GAINS = new Gains(1.2, 0.0, 0.0, 0.0, 0, 0.0);

            // The steer motor uses any SwerveModule.SteerRequestType control request with the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            public static final Slot0Configs PREPROVIDED_STEER_GAINS = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.6)
                .withKS(0.1).withKV(1.91).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            public static final Slot0Configs PREPROVIDED_DRIVE_GAINS = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);
        }
    
        public static final class AutoConstants {
            public static final Gains XY_GAINS = new Gains(3,0,0);
            public static final Gains ROT_GAINS = new Gains(0.05,0,0.0);

                    
            public static final double XY_TOLERANCE = 0.05;
            public static final double ROT_TOLERANCE = 1;
                    
            public static final Pose2d targetpos = new Pose2d(new Translation2d(0.3,0), new Rotation2d());
        }
    
    
        public static final class Configurations {
            public static final double OPEN_LOOP_RAMP_RATE = 0.0; // Todo: Test. think this will help.
            public static final double CLOSED_LOOP_RAMP_RATE = 0.0; // Todo: Test. think this will help.
            public static final double NEUTRAL_DEADBAND = 0.04;

            // POWER! (limiting)
            private static final TalonFXConfiguration DRIVE_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
                ).withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
                ).withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            );
            private static final TalonFXConfiguration STEER_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40) // TODO: tune???
                        .withStatorCurrentLimitEnable(true)
                    ).withMotorOutput(
                        new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
                    // ).withOpenLoopRamps(
                    //     new OpenLoopRampsConfigs()
                    //         .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
                    // ).withClosedLoopRamps(
                    //     new ClosedLoopRampsConfigs()
                    //         .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            );
            private static final double SLIP_CURRENT = 100; // TODO: Tune??? 
        }

        // No mans land
        // Beware, there be dragons.
        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(IDs.DRIVE_PIGEON.id);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>() // holy verbosity batman.
                .withDriveMotorGearRatio(DRIVE_RATIO)
                .withSteerMotorGearRatio(STEER_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS)
                .withSteerMotorGains(PIDConstants.PREPROVIDED_STEER_GAINS) // ?
                .withDriveMotorGains(PIDConstants.PREPROVIDED_DRIVE_GAINS) // ?
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(Configurations.SLIP_CURRENT)
                .withSpeedAt12Volts(MAX_SPEED_MEETERS_PER_SEC)
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(Configurations.DRIVE_MOTOR_CONFIGS)
                .withSteerMotorInitialConfigs(Configurations.STEER_MOTOR_CONFIGS);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT =
            ConstantCreator.createModuleConstants(
                IDs.LEFT_FRONT_STEER.id, IDs.LEFT_FRONT_WHEEL.id, IDs.LEFT_FRONT_ENCODER.id, ModuleSpecificConstants.FRONT_LEFT_ENCODER_OFFSET,
                ModuleSpecificConstants.FRONT_LEFT_XPOS, ModuleSpecificConstants.FRONT_LEFT_YPOS, 
                ModuleSpecificConstants.FRONT_LEFT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_LEFT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT =
            ConstantCreator.createModuleConstants(
                IDs.RIGHT_FRONT_STEER.id, IDs.RIGHT_FRONT_WHEEL.id, IDs.RIGHT_FRONT_ENCODER.id, ModuleSpecificConstants.FRONT_RIGHT_ENCODER_OFFSET,
                ModuleSpecificConstants.FRONT_RIGHT_XPOS, ModuleSpecificConstants.FRONT_RIGHT_YPOS, 
                ModuleSpecificConstants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_RIGHT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_RIGHT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT =
            ConstantCreator.createModuleConstants(
                IDs.LEFT_BACK_STEER.id, IDs.LEFT_BACK_WHEEL.id, IDs.LEFT_BACK_ENCODER.id, ModuleSpecificConstants.BACK_LEFT_ENCODER_OFFSET,
                ModuleSpecificConstants.BACK_LEFT_XPOS, ModuleSpecificConstants.BACK_LEFT_YPOS, 
                ModuleSpecificConstants.BACK_LEFT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.BACK_LEFT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.BACK_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT =
            ConstantCreator.createModuleConstants(
                IDs.RIGHT_BACK_STEER.id, IDs.RIGHT_BACK_WHEEL.id, IDs.RIGHT_BACK_ENCODER.id, ModuleSpecificConstants.BACK_RIGHT_ENCODER_OFFSET,
                ModuleSpecificConstants.BACK_RIGHT_XPOS, ModuleSpecificConstants.BACK_RIGHT_YPOS, 
                ModuleSpecificConstants.BACK_RIGHT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.BACK_RIGHT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.BACK_RIGHT_ENCODER_INVERTED
        );
        
        // misc
        public static final int TIMEOUT_MS = 30;
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
      }
    
    public static final class VisionConstants { 
        public static final String CAMERA_NAME = "Camera_Module_v1";

        public static final Transform3d CAMERA_POS = new Transform3d(new Translation3d(-.3048, 0.2413*0, .2794), new Rotation3d(0,0.52333,Math.PI));
        
        // public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


        // Test april tag field layout
        public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(
            Arrays.asList(new AprilTag[] {
                new AprilTag(1, new Pose3d(
                    new Translation3d(0.,0.,0.26035), new Rotation3d(0.,0.,0.)
                )),
            }), 100, 100);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class DriveConstants {
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    }
    
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 0;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
        public static final int XBOX_PROGRAMMER_ID = 2;
        public static final double LEFT_AXIS_DEADBAND = 0.1;

    }
}