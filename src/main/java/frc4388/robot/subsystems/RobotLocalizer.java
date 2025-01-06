package frc4388.robot.subsystems;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;

public class RobotLocalizer extends Subsystem {
    private RobotGyro gyro;
    private Pose2d lastPose2d = new Pose2d();
    private PhotonCameraSim camera;

    private SwerveModule leftFront;
    private SwerveModule rightFront;
    private SwerveModule leftBack;
    private SwerveModule rightBack;

    private SwerveDriveOdometry physicalOdometry;

    private Field2d smdField = new Field2d();

    public RobotLocalizer(
            SwerveModule leftFront,
            SwerveModule rightFront,
            SwerveModule leftBack,
            SwerveModule rightBack,
        
            RobotGyro gyro, PhotonCameraSim cam
        ) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        this.gyro = gyro;
        this.camera = cam;

        this.physicalOdometry = new SwerveDriveOdometry(
            SwerveDriveConstants.KINEMATICS, 
            gyro.getRotation2d(),
            getSwerveModulePositions(),
            new Pose2d()
        );


        SmartDashboard.putData("Robot Pose", smdField);
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            leftFront.getPosition(),
            rightFront.getPosition(),
            leftBack.getPosition(),
            rightBack.getPosition()
        };
    }

    private Translation3d gyroAccel = new Translation3d();
    private Rotation2d gyroRot = new Rotation2d();

    private Pose2d physicalOdometryPose = new Pose2d();

    @Override
    public void periodic() {
        gyroAccel = gyro.getAcceleration3d();
        gyroRot = gyro.getRotation2d();

        physicalOdometryPose = physicalOdometry.update(gyroRot, getSwerveModulePositions());

        // Translation2d pos;
        
        // pos = lastPose2d.getTranslation();


        lastPose2d = physicalOdometryPose;
    }

    // Pathplanner function
    public Pose2d getPose(){
        return lastPose2d;
    }

    // PathPlanner func
    public void resetPose(Pose2d pose){
        lastPose2d = pose;
    }

    // PathPlanner
    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
            0,
            0,
            Units.rotationsToRadians(gyro.getYawAngularVelocity())
        );
    }

    @Override
    public String getSubsystemName() {
        return "Robot Localizer";
    }


    ShuffleboardLayout subsystemLayout = Shuffleboard.getTab("Subsystems")
    .getLayout("Robot Localizer", BuiltInLayouts.kList)
    .withSize(2, 2);

    GenericEntry sbGyro =  subsystemLayout
    .add("Robot Yaw", 0)
    .withWidget(BuiltInWidgets.kGyro)
    .getEntry();

    GenericEntry sbField =  subsystemLayout
    .add("Acceleration", 0)
    .withWidget(BuiltInWidgets.kField)
    .getEntry();

    @Override
    public void queryStatus() {
        // subsystemLayout

        smdField.setRobotPose(lastPose2d);

        sbGyro.setDouble(gyroRot.getDegrees());
        Shuffleboard.update();
    }

    @Override
    public Status diagnosticStatus() {
        return new Status();
    }
}