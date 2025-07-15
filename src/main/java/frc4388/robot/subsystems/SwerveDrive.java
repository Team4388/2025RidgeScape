// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.constants.DriveConstants;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.utility.compute.TimesNegativeOne;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveDrive extends SubsystemBase implements Queryable {
    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;
    private Vision vision;

    // @AutoLog
    // public class SwerveDriveState {
    public int gear_index = DriveConstants.STARTING_GEAR;
    public boolean stopped = false;
    public boolean robotKnowsWhereItIs = false;

    public double speedAdjust = DriveConstants.MAX_SPEED_MEETERS_PER_SEC * DriveConstants.GEARS[gear_index];
    public double rotSpeedAdjust = DriveConstants.MAX_ROT_SPEED;
    public double autoSpeedAdjust = DriveConstants.MAX_SPEED_MEETERS_PER_SEC * 0.25; // cap auto performance to
                                                                                            // 25%

    public double lastOdomSpeed;

    public Pose2d initalPose2d = null;


    public double rotTarget = 0.0;
    public Rotation2d orientRotTarget = new Rotation2d();
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    // }

    // public SwerveDriveState state = new SwerveDriveState();

    /** Creates a new SwerveDrive. */
    public SwerveDrive(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain, Vision vision) {
        // public SwerveDrive(SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        // swerveDriveTrain) {
        FaultReporter.register(this);

        this.swerveDriveTrain = swerveDriveTrain;
        this.vision = vision;

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            config = null;
        }
        // DoubleSupplier a = () -> 1.d;
        AutoBuilder.configure(
                () -> {
                    return swerveDriveTrain.samplePoseAt(Utils.getCurrentTimeSeconds()).orElse(initalPose2d);
                }, // Robot pose supplier
                this::setOdoPose, // Method to reset odometry (will be called if your auto has a starting
                                             // pose)
                () -> swerveDriveTrain.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> swerveDriveTrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                                              // Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    // var alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    return TimesNegativeOne.isRed;
                },
                this // Reference to this subsystem to set requirements
        );

        PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        
        PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });



        // // Configure SysId
        // sysId =
        //     new SysIdRoutine(
        //         new SysIdRoutine.Config(
        //             null,
        //             null,
        //             null,
        //             (state) -> Logger.recordOutput("Drive/SysIdState", toString())),
        //         new SysIdRoutine.Mechanism(
        //             (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    }

    public void setOdoPose(Pose2d pose) {
        if (pose == null) return;
        initalPose2d = pose;
        swerveDriveTrain.resetPose(pose);
    }

    // public void oneModuleTest(SwerveModule module, Translation2d leftStick,
    // Translation2d rightStick){
    // // double ang = Math.atan2(rightStick.getY(), rightStick.getX());
    // // rightStick.getAngle()
    // double speed = Math.sqrt(Math.pow(leftStick.getX(), 2) +
    // Math.pow(leftStick.getY(), 2));
    // // System.out.println(ang);
    // // module.go(ang);
    // // Rotation2d rot = Rotation2d.fromRadians(ang);
    // Rotation2d rot = new Rotation2d(rightStick.getX(), rightStick.getY());
    // SwerveModuleState state = new SwerveModuleState(speed, rot);
    // module.setDesiredState(state);
    // }

    public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve drive is still going:
            stopModules(); // stop the swerve

        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) // if no imput
            return; // don't bother doing swerve drive math and return early.

        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);
        
        stopped = false;
        if (fieldRelative) {
            
            leftStick = TimesNegativeOne.invert(leftStick, TimesNegativeOne.XAxis, TimesNegativeOne.YAxis);
            rightStick = TimesNegativeOne.invert(rightStick, TimesNegativeOne.RotAxis);    

            // ! drift correction
            if (rightStick.getNorm() > 0.05 || !DriveConstants.DRIFT_CORRECTION_ENABLED) {
                rotTarget = swerveDriveTrain.samplePoseAt(Utils.getCurrentTimeSeconds()).orElse(new Pose2d()).getRotation().getDegrees();
                swerveDriveTrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(leftStick.getY() * speedAdjust)
                    .withRotationalRate(rightStick.getX() * rotSpeedAdjust));
                    // .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
                SmartDashboard.putBoolean("drift correction", false);
            } else {
                var ctrl = new SwerveRequest.FieldCentricFacingAngle()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(leftStick.getY() * speedAdjust)
                    .withTargetDirection(Rotation2d.fromDegrees(rotTarget));
                ctrl.HeadingController.setPID(
                    DriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kP,
                    DriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kI,
                    DriveConstants.PIDConstants.DRIFT_CORRECTION_GAINS.kD
                );
                swerveDriveTrain.setControl(ctrl);
                SmartDashboard.putBoolean("drift correction", true);
            }

           
        } else { // Create robot-relative speeds.
            swerveDriveTrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(leftStick.getX() * speedAdjust)
                    .withVelocityY(-leftStick.getY() * speedAdjust)
                    .withRotationalRate(rightStick.getX() * rotSpeedAdjust));
        }
    }

    public void driveFine(Translation2d leftStick, Translation2d rightStick, double percentOutput) {
        stopped = false;
        // Create robot-relative speeds.
        if (rightStick.getNorm() > 0.1) rightStick = rightStick.times(0);
        swerveDriveTrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(leftStick.getX() * DriveConstants.MAX_SPEED_MEETERS_PER_SEC * percentOutput)
            .withVelocityY(-leftStick.getY() * DriveConstants.MAX_SPEED_MEETERS_PER_SEC * percentOutput)
            .withRotationalRate(rightStick.getX() * rotSpeedAdjust));
        
    }


    public void driveWithInputOrientation(Translation2d leftStick, Translation2d rightStick) { // there is no practical
                                                                                               // reason to have a robot
                                                                                               // relitive version of
                                                                                               // this, and no pre
                                                                                               // provided version
        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the swerve
                                                                                           // drive is still going:
            stopModules(); // stop the swerve

        if (rightStick.getNorm() < 0.05 && leftStick.getNorm() < 0.05) // if no imput
            return; // don't bother doing swerve drive math and return early.

        leftStick.rotateBy(TimesNegativeOne.ForwardOffset);

        swerveDriveTrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(leftStick.getX() * speedAdjust)
                .withVelocityY(leftStick.getY() * speedAdjust)
                .withTargetDirection(rightStick.getAngle()));
    }

    public void driveRelativeAngle(Translation2d leftStick, Rotation2d heading) {
        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);
        leftStick = TimesNegativeOne.invert(leftStick, TimesNegativeOne.XAxis, TimesNegativeOne.YAxis);
        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        ctrl.HeadingController.setPID(
            DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kP,
            DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kI,
            DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kD
        );
        swerveDriveTrain.setControl(ctrl);
    }

    public void driveRelativeLockedAngle(Translation2d leftStick, Rotation2d heading) {
        leftStick = leftStick.rotateBy(heading);

        var ctrl = new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(leftStick.getX() * speedAdjust)
            .withVelocityY(leftStick.getY() * speedAdjust)
            .withTargetDirection(heading);
        // ctrl.HeadingController.setPID(
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kP,
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kI,
        //     DriveConstants.PIDConstants.RELATIVE_LOCKED_ANGLE_GAINS.kD
        // );
        swerveDriveTrain.setControl(ctrl);
    }

    public void setLimits(double limitInAmps) {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerveDriveTrain.getModules()) {
            var talonFXConfigurator = module.getDriveMotor().getConfigurator();
            var talonFXConfigs = new TalonFXConfiguration();

            talonFXConfigurator.refresh(talonFXConfigs);
            talonFXConfigs.CurrentLimits.StatorCurrentLimit = limitInAmps;
            talonFXConfigs.CurrentLimits.SupplyCurrentLimit = limitInAmps+10;
            talonFXConfigurator.apply(talonFXConfigs);
        }
    }

    public void activateLuigiMode() {
        setLimits(20);
    }

    public void deactivateLuigiMode() {
        setLimits(DriveConstants.Configurations.SLIP_CURRENT);
    }

    public boolean rotateToTarget(double angle) {
        swerveDriveTrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromDegrees(angle)));

        if (Math.abs(angle - getGyroAngle()) < 5.0) {
            return true;
        }

        return false;
    }

    public boolean isStopped() {
        return lastOdomSpeed < AutoConstants.STOP_VELOCITY;
    }

    public void driveWithInputRotation(Translation2d leftStick, Rotation2d rot) {
        // if (leftStick.getNorm() < 0.05 && stopped == false) // if no imput and the
        // swerve drive is still going:
        // stopModules(); // stop the swerve

        // if (leftStick.getNorm() < 0.05) //if no imput
        // return; // don't bother doing swerve drive math and return early.

        leftStick = leftStick.rotateBy(TimesNegativeOne.ForwardOffset);

        swerveDriveTrain.setControl(new SwerveRequest.FieldCentricFacingAngle()
                .withVelocityX(leftStick.getX() * -speedAdjust)
                .withVelocityY(leftStick.getY() * speedAdjust)
                .withTargetDirection(rot));
        // double
    }

    public double getGyroAngle() {
        return getPose2d().getRotation().getRadians();
    }

    public Pose2d getPose2d() {
        return swerveDriveTrain.samplePoseAt(Vision.getTime()).orElse(initalPose2d);
    }

    public void resetGyro() {
        swerveDriveTrain.tareEverything();
        robotKnowsWhereItIs = false;
        rotTarget = 0;
        // vision.resetRotations();
    }


    public void softStop() {
        stopped = true;
        swerveDriveTrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        ); // stop the modules without breaking
    }

    public void stopModules() {
        // stopped = true;
        // swerveDriveTrain.setControl(new SwerveRequest.SwerveDriveBrake());
        softStop();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run\
        SmartDashboard.putNumber("Gyro", (getGyroAngle() * 180) / Math.PI);
        SmartDashboard.putNumber("RotTartget", rotTarget);

        double time = Vision.getTime();
        double freq =  swerveDriveTrain.getOdometryFrequency();

        Optional<Pose2d> curpose = swerveDriveTrain.samplePoseAt(time);
        Optional<Pose2d> lastPose = swerveDriveTrain.samplePoseAt(time - freq);
        
        vision.setLastOdomPose(curpose);
        setLastOdomSpeed(curpose, lastPose, freq);

        if (vision.isTag()) {
            Pose2d pose = vision.getPose2d();
            if (!robotKnowsWhereItIs) {
                robotKnowsWhereItIs = true;
                Pose2d curPose = getPose2d();
                rotTarget += pose.getRotation().getDegrees() - curPose.getRotation().getDegrees();
            }

            vision.addVisionMeasurement(swerveDriveTrain);
        }

        // if(e.isPresent())
    }

    private void reset_index() {
        gear_index = DriveConstants.STARTING_GEAR; // however we wish to initialize the gear (What gear does the
                                                         // robot start in?)
    }

    public void shiftDown() {
        if (gear_index == -1 || gear_index >= DriveConstants.GEARS.length)
            reset_index(); // If outof bounds: reset index
        int i = gear_index - 1;
        if (i == -1)
            i = 0;
        setPercentOutput(DriveConstants.GEARS[i]);
        gear_index = i;
    }

    public void shiftUp() {
        if (gear_index == -1 || gear_index >= DriveConstants.GEARS.length)
            reset_index(); // If outof bounds: reset index
        int i = gear_index + 1;
        if (i == DriveConstants.GEARS.length)
            i = DriveConstants.GEARS.length - 1;
        setPercentOutput(DriveConstants.GEARS[i]);
        gear_index = i;
    }

    public void setPercentOutput(double speed) {
        speedAdjust = DriveConstants.MAX_SPEED_MEETERS_PER_SEC * speed;
        gear_index = -1;
    }

    public void setToSlow() {
        setPercentOutput(DriveConstants.SLOW_SPEED);
        gear_index = 0;
    }

    public void setToFast() {
        setPercentOutput(DriveConstants.FAST_SPEED);
        gear_index = 1;
    }

    public void setToTurbo() {
        setPercentOutput(DriveConstants.TURBO_SPEED);
        gear_index = 2;
    }

    public void shiftUpRot() {
        rotSpeedAdjust = DriveConstants.ROTATION_SPEED;
    }

    public void shiftDownRot() {
        rotSpeedAdjust = DriveConstants.MIN_ROT_SPEED;
    }

    private int tmp_gear_index = DriveConstants.STARTING_GEAR;

    public void startSlowPeriod() {
        tmp_gear_index = gear_index;
        setToSlow();
    }

    public void startTurboPeriod() {
        tmp_gear_index = gear_index;
        setToTurbo();
    }

    public void endSlowPeriod() {
        setPercentOutput(DriveConstants.GEARS[tmp_gear_index]);
        gear_index = tmp_gear_index;
    }



    public void setLastOdomSpeed(Optional<Pose2d> curPose, Optional<Pose2d> lastPose, double freq){
        if(curPose.isPresent() && lastPose.isPresent()){
            lastOdomSpeed = curPose.get().getTranslation().getDistance(lastPose.get().getTranslation())/freq;
        }
    }
    


    @Override
    public String getName() {
        return "Swerve Drive Controller";
    }

    @Override
    public Status diagnosticStatus() {
        Status status = new Status();

        // status.addReport(ReportLevel.INFO,
        //         "Don't know how to diganose new CTRE swerve systems. please check under the CAN(t) section for more detailed information about the swerves there.");

        return status;
    }


    // Update CTRE simulation, if used.
    public void updateSim(double voltage) {
        swerveDriveTrain.updateSimState(0.02, voltage);
    }
}

