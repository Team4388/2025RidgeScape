// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.logging.Level;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.Constants.SwerveDriveConstants.Conversions;
import frc4388.utility.RobotGyro;
import frc4388.utility.RobotUnits;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.Status.Report;
import frc4388.utility.Status.ReportLevel;

public class SwerveDrive extends Subsystem {
  private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;
  
  private int gear_index;
  private boolean stopped = false;

  public double speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
  public double rotSpeedAdjust = SwerveDriveConstants.MAX_ROT_SPEED;
  public double autoSpeedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  
  public double rotTarget = 0.0;
  public Rotation2d orientRotTarget = new Rotation2d();
  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain) {
    super();
    
    this.swerveDriveTrain = swerveDriveTrain;

    reset_index();
  }

  public void oneModuleTest(SwerveModule module, Translation2d leftStick, Translation2d rightStick){
    // double ang = Math.atan2(rightStick.getY(), rightStick.getX());
    // rightStick.getAngle()
    double speed = Math.sqrt(Math.pow(leftStick.getX(), 2) + Math.pow(leftStick.getY(), 2));
    // System.out.println(ang);
    // module.go(ang);
    // Rotation2d rot = Rotation2d.fromRadians(ang);
    Rotation2d rot = new Rotation2d(rightStick.getX(), rightStick.getY());
    SwerveModuleState state = new SwerveModuleState(speed, rot);
    module.setDesiredState(state);
  }

  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    leftStick.rotateBy(Rotation2d.fromDegrees(SwerveDriveConstants.FORWARD_OFFSET));
    if (fieldRelative) {
      // if (rightStick.getNorm() > 0.05 && 
      swerveDriveTrain.setControl(new SwerveRequest.FieldCentric()
        .withVelocityX(leftStick.getX()*speedAdjust)
        .withVelocityY(leftStick.getY()*speedAdjust)
        .withRotationalRate(rightStick.getY()*rotSpeedAdjust)
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
      chassisSpeeds = new ChassisSpeeds(-1 * leftStick.getX(), -1 * leftStick.getY(), -1 * rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED);
    }
    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void playbackDriveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (fieldRelative) {

      double rot = 0;
      
      // ! drift correction
      if (rightStick.getNorm() > 0.05) {
        rotTarget = gyro.getAngle();
        rot = rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED;
      //  SmartDashboard.putBoolean("drift correction", false);
        stopped = false;
      } else if(leftStick.getNorm() > 0.05) {
        if (!stopped) {
          stopModules();
          stopped = true;
        }

     //   SmartDashboard.putBoolean("drift correction", true);
        // double rot_correction = ((rotTarget - gyro.getAngle()) / 360) * SwerveDriveConstants.ROT_CORRECTION_SPEED;
        

      }

      // Use the left joystick to set speed. Apply a cubic curve and the set max speed.
      Translation2d speed = leftStick.times(leftStick.getNorm() * autoSpeedAdjust);
      // Translation2d cubedSpeed = new Translation2d(Math.pow(speed.getX(), 3.00), Math.pow(speed.getY(), 3.00));

      // Convert field-relative speeds to robot-relative speeds.
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * speed.getX(), -1 * speed.getY(), rightStick.getX() * SwerveDriveConstants.PLAYBACK_ROTATION_SPEED, gyro.getRotation2d());//.times(-1));
    } else {      // Create robot-relative speeds.
      chassisSpeeds = new ChassisSpeeds(-1 * leftStick.getX(), -1 * leftStick.getY(), -1 * rightStick.getX() * SwerveDriveConstants.PLAYBACK_ROTATION_SPEED);
    }
    // setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void driveWithInputOrientation(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {

    // Translation2d rightStick = new Translation2d(-rightX, rightY);
    double rightX = rightStick.getX();
    double rightY = rightStick.getY();

    double rot_correction = 0;

    // double rot_correction = ((rightStick.getAngle().getDegrees() - gyro.getAngle()) / 360) * SwerveDriveConstants.ROT_CORRECTION_SPEED;

    if(fieldRelative) {
      double rot = 0;
      if(rightStick.getNorm() > 0.5) {
        orientRotTarget = new Rotation2d(rightX, -rightY).minus(new Rotation2d(0,1));
        
        Rotation2d tmp =  orientRotTarget.minus(gyro.getRotation2d().minus(new Rotation2d(Math.PI)).interpolate(orientRotTarget, 0.5));
        double min = tmp.getDegrees();
        min = Math.max(Math.abs(min), 2);
        if(tmp.getDegrees() < 0)
          min*=-1;
        tmp = new Rotation2d(min * Math.PI / 180);
        rot = tmp.getRadians(); // x   x - y/x
      }
    
    Translation2d speed = leftStick.times(leftStick.getNorm() * speedAdjust);

      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * speed.getX(), -1 * speed.getY(), ((-1 * rightStick.getX()) * SwerveDriveConstants.ROTATION_SPEED) + rot_correction, gyro.getRotation2d()).times(1);
    } else {      // Create robot-relative speeds.
      chassisSpeeds = new ChassisSpeeds(-1 * leftStick.getX(), -1 * leftStick.getY(), -1 * rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED);
    }
    // setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));  
    }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state);
    }
  }

  public boolean rotateToTarget(double angle) {
    double currentAngle = getGyroAngle();
    double error = angle - currentAngle;

    driveWithInput(new Translation2d(0, 0), new Translation2d(error / Math.abs(error) * 0.3, 0), true);

    if (Math.abs(angle - getGyroAngle()) < 5.0) {
      return true;
    }

    return false;
  }

  public double getGyroAngle() {
    return -gyro.getAngle();
  }

  public void add180() {
    gyro.reset(gyro.getAngle()+180);
    rotTarget = gyro.getAngle();

  }

  public void resetGyro() {
    gyro.reset();
    rotTarget = gyro.getAngle();
  }

  public void resetGyroFlip() {
    gyro.resetFlip();
    rotTarget = gyro.getAngle();
  }

  public void resetGyroRightBlue() {
    gyro.resetRightSideBlue();
    rotTarget = gyro.getAngle();
  }

  public void resetGyroRightAmp() {
    gyro.resetAmpSide();
    rotTarget = gyro.getAngle();
  }
  
  public void stopModules() {
    for (SwerveModule module : this.modules) {
      module.stop();
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  public boolean getSpeedState() {
    
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("RotTartget", rotTarget);
  }

  private void reset_index() {
    gear_index = 0; // however we wish to initialize the gear (What gear does the robot start in?)
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
    speedAdjust = Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST * speed;
    gear_index = -1;
  }

  public void setToSlow() {
    this.speedAdjust = Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST * SwerveDriveConstants.SLOW_SPEED;
    for(int i=0;i<5;i++){
      Log("SLOW");
    }
  }

  public void setToFast() {
    this.speedAdjust = Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST * SwerveDriveConstants.FAST_SPEED;
    for(int i=0;i<5;i++){
      Log("FAST");
    }
  }

  public void setToTurbo() {
    this.speedAdjust = Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST * SwerveDriveConstants.TURBO_SPEED;
        for(int i=0;i<5;i++){
      Log("TURBO");
    }
  }

  public void toggleGear(double angle) {
    if (Math.abs(this.speedAdjust - SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW) < .01 && Math.abs(angle) < 10) {
      this.speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
      SwerveDriveConstants.ROT_CORRECTION_SPEED = SwerveDriveConstants.CORRECTION_MIN;
    } else {
      this.speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
      SwerveDriveConstants.ROT_CORRECTION_SPEED = SwerveDriveConstants.CORRECTION_MIN;
    }
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
    sbGyro.setDouble(this.gyro.getAngle());
    sbShiftState.setDouble(this.speedAdjust);
  
    //TODO: Add more status things
  }

  @Override
  public Status diagnosticStatus() {
    Status status = new Status();
    for (SwerveModule module : modules) {
      for (Report moduleDignostic : module.diagnosticStatus().reports) {
        status.addReport(moduleDignostic.reportLevel, "[" + module.getSubsystemName() + "] " + moduleDignostic.description);
      }
    }

    status.diagnoseHardwareCTRE("Swerve Gyro", gyro.getPigeon());
    
    return status;
  }
}
