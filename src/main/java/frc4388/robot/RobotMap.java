/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import frc4388.robot.Constants.LEDConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.subsystems.SwerveModule;
import frc4388.utility.RobotGyro;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    // private Pigeon2 m_pigeon2 = new Pigeon2(SwerveDriveConstants.IDs.DRIVE_PIGEON.id);
    // public RobotGyro gyro = new RobotGyro(m_pigeon2);
    
    public SwerveModule leftFront;
    public SwerveModule rightFront;
    public SwerveModule leftBack;
    public SwerveModule rightBack;

    public RobotMap() {
        configureDriveMotorControllers();
    }

    /* LED Subsystem */
    // public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);

    public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(
        TalonFX::new, TalonFX::new, CANcoder::new, 
        new SwerveDrivetrainConstants().withPigeon2Id(SwerveDriveConstants.IDs.DRIVE_PIGEON.id),
        new SwerveModuleConstants[] {
        }
    )

    /* Swreve Drive Subsystem */
    public final TalonFX  leftFrontWheel    = new  TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_WHEEL.id);
    public final TalonFX  leftFrontSteer    = new  TalonFX(SwerveDriveConstants.IDs.LEFT_FRONT_STEER.id);
    public final CANcoder leftFrontEncoder  = new CANcoder(SwerveDriveConstants.IDs.LEFT_FRONT_ENCODER.id);


    public final TalonFX  rightFrontWheel   = new  TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_WHEEL.id);
    public final TalonFX  rightFrontSteer   = new  TalonFX(SwerveDriveConstants.IDs.RIGHT_FRONT_STEER.id);
    public final CANcoder rightFrontEncoder = new CANcoder(SwerveDriveConstants.IDs.RIGHT_FRONT_ENCODER.id);
    
    public final TalonFX  leftBackWheel     = new  TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_WHEEL.id);
    public final TalonFX  leftBackSteer     = new  TalonFX(SwerveDriveConstants.IDs.LEFT_BACK_STEER.id);
    public final CANcoder leftBackEncoder   = new CANcoder(SwerveDriveConstants.IDs.LEFT_BACK_ENCODER.id);
    
    public final TalonFX  rightBackWheel    = new  TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_WHEEL.id);
    public final TalonFX  rightBackSteer    = new  TalonFX(SwerveDriveConstants.IDs.RIGHT_BACK_STEER.id);
    public final CANcoder rightBackEncoder  = new CANcoder(SwerveDriveConstants.IDs.RIGHT_BACK_ENCODER.id);

    void configureDriveMotorControllers() {
        // initialize SwerveModules
        this.rightFront = new SwerveModule("Right Front Swerve Module", rightFrontWheel, rightFrontSteer, rightFrontEncoder, SwerveDriveConstants.DefaultSwerveRotOffsets.FRONT_RIGHT_ROT_OFFSET);
        this.leftFront  = new SwerveModule("Left Front Swerve Module",  leftFrontWheel,  leftFrontSteer,  leftFrontEncoder,  SwerveDriveConstants.DefaultSwerveRotOffsets.FRONT_LEFT_ROT_OFFSET);
        this.leftBack   = new SwerveModule("Left Back Swerve Module",   leftBackWheel,   leftBackSteer,   leftBackEncoder,   SwerveDriveConstants.DefaultSwerveRotOffsets.BACK_LEFT_ROT_OFFSET);
        this.rightBack  = new SwerveModule("Right Back Swerve Module",  rightBackWheel,  rightBackSteer,  rightBackEncoder,  SwerveDriveConstants.DefaultSwerveRotOffsets.BACK_RIGHT_ROT_OFFSET);
    }
}
