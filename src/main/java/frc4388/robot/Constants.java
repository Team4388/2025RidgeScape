/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc4388.utility.CanDevice;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;

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
        
        public static final double[] GEARS = {0.25, 0.5, 1.0};

        public static final double SLOW_SPEED = 0.25;
        public static final double FAST_SPEED = 0.5;
        public static final double TURBO_SPEED = 1.0;

        // dimensions
        public static final double WIDTH = 18.5;
        public static final double HEIGHT = 18.5;
        public static final double HALF_WIDTH = WIDTH / 2.d;
        public static final double HALF_HEIGHT = HEIGHT / 2.d;

        private static final class ModuleSpecificConstants {
            //Front Left
            private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.0);
            private static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = false;
            private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;
            private static final Distance FRONT_LEFT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance FRONT_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            
            //Front Right
            private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(0.0);
            private static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = false;
            private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;
            private static final Distance FRONT_RIGHT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance FRONT_RIGHT_YPOS = Inches.of(HALF_HEIGHT);

            //Back Left
            private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.0);
            private static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = false;
            private static final boolean BACK_LEFT_ENCODER_INVERTED = false;
            private static final Distance BACK_LEFT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance BACK_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            
            //Back Right
            private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(0.0);
            private static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = false;
            private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;
            private static final Distance BACK_RIGHT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance BACK_RIGHT_YPOS = Inches.of(HALF_HEIGHT);
        }

        private static final class DrivetrainSpecificConstants {
            
        }

        public static final class IDs {
            public static final CanDevice RIGHT_FRONT_WHEEL   = new CanDevice("RIGHT_FRONT_WHEEL", 2);
            public static final CanDevice RIGHT_FRONT_STEER   = new CanDevice("RIGHT_FRONT_STEER", 3);
            public static final CanDevice RIGHT_FRONT_ENCODER = new CanDevice("RIGHT_FRONT_ENCODER", 10);
            
            public static final CanDevice LEFT_FRONT_WHEEL    = new CanDevice("LEFT_FRONT_WHEEL", 4);
            public static final CanDevice LEFT_FRONT_STEER    = new CanDevice("LEFT_FRONT_STEER", 5);
            public static final CanDevice LEFT_FRONT_ENCODER  = new CanDevice("LEFT_FRONT_ENCODER", 11);
            
            public static final CanDevice LEFT_BACK_WHEEL     = new CanDevice("LEFT_BACK_WHEEL", 6);
            public static final CanDevice LEFT_BACK_STEER     = new CanDevice("LEFT_BACK_STEER", 7);
            public static final CanDevice LEFT_BACK_ENCODER   = new CanDevice("LEFT_BACK_ENCODER", 12);
            
            public static final CanDevice RIGHT_BACK_WHEEL    = new CanDevice("RIGHT_BACK_WHEEL", 8);  
            public static final CanDevice RIGHT_BACK_STEER    = new CanDevice("RIGHT_BACK_STEER", 9);
            public static final CanDevice RIGHT_BACK_ENCODER  = new CanDevice("RIGHT_BACK_ENCODER", 13);

            public static final CanDevice DRIVE_PIGEON        = new CanDevice("DRIVE_PIGEON", 14);
            public static final CanDevice e        = new CanDevice("NONEXISTANT_CAN", 50);
        }
    
        public static final class PIDConstants {
            public static final int SWERVE_SLOT_IDX = 0;
            public static final int SWERVE_PID_LOOP_IDX = 1;
            public static final Gains SWERVE_GAINS = new Gains(50, 0.0, 0.32, 0.0, 0, 0.0);

            public static final Gains TEST_SWERVE_GAINS = new Gains(1.2, 0.0, 0.0, 0.0, 0, 0.0);

        }
    
        public static final class AutoConstants {
            public static final Gains X_CONTROLLER = new Gains(0.8, 0.0, 0.0);
            public static final Gains Y_CONTROLLER = new Gains(0.8, 0.0, 0.0);
            public static final Gains THETA_CONTROLLER = new Gains(-0.8, 0.0, 0.0);
            public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI/2, Math.PI/2); // TODO: tune
            
            public static final double PATH_MAX_VEL = 0.3; // TODO: find the actual value
            public static final double PATH_MAX_ACC = 0.3; // TODO: find the actual value
        }
    
        public static final class Conversions {
            public static final double JOYSTICK_TO_METERS_PER_SECOND_FAST = 6.22;
            public static final double JOYSTICK_TO_METERS_PER_SECOND_SLOW = JOYSTICK_TO_METERS_PER_SECOND_FAST * 0.5;
        
            public static final double MOTOR_REV_PER_WHEEL_REV = 5.12;
            public static final double MOTOR_REV_PER_STEER_REV = 12.8;
        
            public static final double TICKS_PER_MOTOR_REV = 0.5;
            public static final double WHEEL_DIAMETER_INCHES = 3.9;
            public static final double INCHES_PER_WHEEL_REV = WHEEL_DIAMETER_INCHES * Math.PI;
        
            public static final double WHEEL_REV_PER_MOTOR_REV = 1 / MOTOR_REV_PER_WHEEL_REV;
            public static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_REV_PER_WHEEL_REV;
            public static final double TICKS_PER_INCH = TICKS_PER_WHEEL_REV / INCHES_PER_WHEEL_REV;
            public static final double INCHES_PER_TICK = 1 / TICKS_PER_INCH;
        
            public static final double TICK_TIME_TO_SECONDS = 10;
            public static final double SECONDS_TO_TICK_TIME = 1 / TICK_TIME_TO_SECONDS;
        }
    
        public static final class Configurations {
            public static final double OPEN_LOOP_RAMP_RATE = 0.2;
            public static final double CLOSED_LOOP_RAMP_RATE = 0.2;
            public static final double NEUTRAL_DEADBAND = 0.04;
        }
    
        public static final double MAX_SPEED_FEET_PER_SECOND = 20.4;
        public static final double MAX_ANGULAR_SPEED_FEET_PER_SECOND = 2 * 2 * Math.PI;
    
       
        

        // No mans land
        // Beware, there be dragons.
        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(1.91).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrent = 120; // TODO: tune???

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(100) // TODO: tune???
                    .withStatorCurrentLimitEnable(true)
            );
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(40) // TODO: tune???
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12Volts = 4.69;// todo: tune???

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.8181818181818183; //todo: find

        private static final double kDriveGearRatio = 6.12;
        private static final double kSteerGearRatio = (150/7);
        private static final Distance kWheelRadius = Inches.of(2); // 2 inches in meters

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(IDs.DRIVE_PIGEON.id);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>() // holy verbosity batman.
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs);

        // Front Left
        private static final int kFrontLeftDriveMotorId = 3;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 10;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(10);
        private static final Distance kFrontLeftYPos = Inches.of(10);

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 0;
        private static final int kFrontRightEncoderId = 0;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(10);
        private static final Distance kFrontRightYPos = Inches.of(-10);

        // Back Left
        private static final int kBackLeftDriveMotorId = 7;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 3;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-10);
        private static final Distance kBackLeftYPos = Inches.of(10);

        // Back Right
        private static final int kBackRightDriveMotorId = 5;
        private static final int kBackRightSteerMotorId = 4;
        private static final int kBackRightEncoderId = 2;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.17236328125);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-10);
        private static final Distance kBackRightYPos = Inches.of(-10);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, Rotations.of(SwerveRotOffsets.FRONT_LEFT_ROT_OFFSET),
                Inches.of(HALF_WIDTH), Inches.of(HALF_HEIGHT), false, false, false
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, false, false, false
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, false, false, false
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        // misc
        public static final int TIMEOUT_MS = 30;
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
      }
    
    public static final class VisionConstants {
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
