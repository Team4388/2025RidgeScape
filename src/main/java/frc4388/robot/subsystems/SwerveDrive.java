// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.Constants.SwerveDriveConstants;

import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.Status.ReportLevel;

public class SwerveDrive extends Subsystem {
  private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;
  
  private int gear_index = SwerveDriveConstants.STARTING_GEAR;
  private boolean stopped = false;

  public double speedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * SwerveDriveConstants.GEARS[gear_index];
  public double rotSpeedAdjust = SwerveDriveConstants.MAX_ROT_SPEED;
  public double autoSpeedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * 0.25; // cap auto performance to 25%
  
  public double rotTarget = 0.0;
  public Rotation2d orientRotTarget = new Rotation2d();
  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  private Field2d field = new Field2d();

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain) {
    super();

    SmartDashboard.putData(field);

    this.swerveDriveTrain = swerveDriveTrain;
  }
  // public void oneModuleTest(SwerveModule module, Translation2d leftStick, Translation2d rightStick){
  //   // double ang = Math.atan2(rightStick.getY(), rightStick.getX());
  //   // rightStick.getAngle()
  //   double speed = Math.sqrt(Math.pow(leftStick.getX(), 2) + Math.pow(leftStick.getY(), 2));
  //   // System.out.println(ang);
  //   // module.go(ang);
  //   // Rotation2d rot = Rotation2d.fromRadians(ang);
  //   Rotation2d rot = new Rotation2d(rightStick.getX(), rightStick.getY());
  //   SwerveModuleState state = new SwerveModuleState(speed, rot);
  //   module.setDesiredState(state);
  // }

  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve drive is still going:
      stopModules(); // stop the swerve
    
    if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) //if no imput
      return; // don't bother doing swerve drive math and return early.

    leftStick = leftStick.rotateBy(Rotation2d.fromDegrees(SwerveDriveConstants.FORWARD_OFFSET));
    
    if (fieldRelative) {
      swerveDriveTrain.setControl(new SwerveRequest.FieldCentric()
        .withVelocityX(leftStick.getX()*speedAdjust)
        .withVelocityY(leftStick.getY()*speedAdjust)
        .withRotationalRate(rightStick.getX()*rotSpeedAdjust)
      );
      // double rot = 0;
      
      // ! drift correction
      // dependant on if the new odomitry system acounts for rotational drift, this may not be needed.
    //   if (rightStick.getNorm() > 0.05) {
    //     rotTarget = swerveDriveTrain.getRotation3d().toRotation2d().getDegrees();
    //     swerveDriveTrain.setControl(new SwerveRequest.FieldCentric()
    //       .withVelocityX(leftStick.getX()*speedAdjust)
    //       .withVelocityY(leftStick.getY()*speedAdjust)
    //       .withRotationalRate(rightStick.getY()*rotSpeedAdjust)
    //     );
      
    //   //  SmartDashboard.putBoolean("drift correction", false);
    //     stopped = false;
    //   } else if(leftStick.getNorm() > 0.05) {
    //     if (!stopped) {
    //       stopModules();
    //       stopped = true;
    //     }

    //  //   SmartDashboard.putBoolean("drift correction", true);
        
    //     // rot = ((rotTarget - gyro.getAngle()) / 360) * SwerveDriveConstants.ROT_CORRECTION_SPEED;

    //   }

    //   // Use the left joystick to set speed. Apply a cubic curve and the set max speed.
    //   Translation2d speed = leftStick.times(leftStick.getNorm() * speedAdjust);
    //   // Translation2d cubedSpeed = new Translation2d(Math.pow(speed.getX(), 3.00), Math.pow(speed.getY(), 3.00));

    //   // Convert field-relative speeds to robot-relative speeds.
    //   // chassisSpeeds = chassisSpeeds.
    //   chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * speed.getX(), -1 * speed.getY(), (-1 * rightStick.getX() * rotSpeedAdjust) - rot_correction, gyro.getRotation2d().times(-1));
    } else {      // Create robot-relative speeds.
      swerveDriveTrain.setControl(new SwerveRequest.RobotCentric()
        .withVelocityX(leftStick.getX()*-speedAdjust)
        .withVelocityY(leftStick.getY()*speedAdjust)
        .withRotationalRate(rightStick.getX()*rotSpeedAdjust)
      );
    }
  }

  public void driveWithInputOrientation(Translation2d leftStick, Translation2d rightStick) { // there is no practical reason to have a robot relitive version of this, and no pre provided version 
    if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve drive is still going:
      stopModules(); // stop the swerve
  
    if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) //if no imput
      return; // don't bother doing swerve drive math and return early.

    leftStick.rotateBy(Rotation2d.fromDegrees(SwerveDriveConstants.FORWARD_OFFSET));
    
    swerveDriveTrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
      .withVelocityX(leftStick.getX()*speedAdjust)
      .withVelocityY(leftStick.getY()*speedAdjust)
      .withTargetDirection(rightStick.getAngle())
    ); 
  }

  public boolean rotateToTarget(double angle) {
    swerveDriveTrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
      .withVelocityX(0)
      .withVelocityY(0)
      .withTargetDirection(Rotation2d.fromDegrees(angle))
    );

    if (Math.abs(angle - getGyroAngle()) < 5.0) {
      return true;
    }

    return false;
  }

  public double getGyroAngle() {
    return swerveDriveTrain.getRotation3d().getAngle();
  }

  public void resetGyro() {
    swerveDriveTrain.tareEverything();
  }

  public void stopModules() {
    // swerveDriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("RotTartget", rotTarget);

    Optional<Pose2d> e = swerveDriveTrain.samplePoseAt(Utils.getCurrentTimeSeconds());

    if(e.isPresent())
      field.setRobotPose(e.get());
  }

  private void reset_index() {
    gear_index = SwerveDriveConstants.STARTING_GEAR; // however we wish to initialize the gear (What gear does the robot start in?)
  }

  public void shiftDown() {
    if (gear_index == -1 || gear_index >= SwerveDriveConstants.GEARS.length) reset_index(); // If outof bounds: reset index
    int i = gear_index - 1;
    if (i == -1) i = 0;
    setPercentOutput(SwerveDriveConstants.GEARS[i]);
    gear_index = i;
  }

  public void shiftUp() {
    if (gear_index == -1 || gear_index >= SwerveDriveConstants.GEARS.length) reset_index(); // If outof bounds: reset index
    int i = gear_index + 1;
    if (i == SwerveDriveConstants.GEARS.length) i = SwerveDriveConstants.GEARS.length - 1;
    setPercentOutput(SwerveDriveConstants.GEARS[i]);
    gear_index = i;
  }

  public void setPercentOutput(double speed) {
    speedAdjust = SwerveDriveConstants.MAX_SPEED_MEETERS_PER_SEC * speed;
    gear_index = -1;
  }

  public void setToSlow() {
    setPercentOutput(SwerveDriveConstants.SLOW_SPEED);
    gear_index = 0;
  }

  public void setToFast() {
    setPercentOutput(SwerveDriveConstants.FAST_SPEED);
    gear_index = 1;
  }

  public void setToTurbo() {
    setPercentOutput(SwerveDriveConstants.TURBO_SPEED);
    gear_index = 2;
  }

  public void shiftUpRot() {
    rotSpeedAdjust = SwerveDriveConstants.ROTATION_SPEED;
  }

  public void shiftDownRot() {
    rotSpeedAdjust = SwerveDriveConstants.MIN_ROT_SPEED;
  }

  @Override
  public String getSubsystemName() {
    return "Swerve Drive Controller";
  }

  ShuffleboardLayout subsystemLayout = Shuffleboard.getTab("Subsystems")
  .getLayout(getSubsystemName(), BuiltInLayouts.kList)
  .withSize(2, 2);

  GenericEntry sbGyro = subsystemLayout
  .add("Gyro angle", 0)
  .withWidget(BuiltInWidgets.kGyro)
  .getEntry();

  GenericEntry sbShiftState = subsystemLayout
  .add("Shift State", 0)
  .withWidget(BuiltInWidgets.kNumberBar)
  .getEntry();

  @Override
  public void queryStatus() {
    sbGyro.setDouble(getGyroAngle());
    sbShiftState.setDouble(this.speedAdjust);
  
    //TODO: Add more status things
  }

  @Override
  public Status diagnosticStatus() {
    Status status = new Status();

    status.addReport(ReportLevel.ERROR, "Don't know how to diganose new CTRE swerve systems. please check under the CAN(t) section for more detailed information about the swerves there.");
    
    return status;
  }
}
