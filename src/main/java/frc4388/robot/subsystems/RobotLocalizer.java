package frc4388.robot.subsystems;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.RobotGyro;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;

public class RobotLocalizer extends Subsystem {
    private RobotGyro gyro;
    private Pose2d lastPose2d = new Pose2d();
    private PhotonCameraSim camera;

    private Translation3d accel = new Translation3d();
    private Rotation3d rot = new Rotation3d();


    public RobotLocalizer(RobotGyro gyro, PhotonCameraSim cam) {
        this.gyro = gyro;
        this.camera = cam;
    }

    @Override
    public void periodic() {
        // time
        accel = gyro.getAcceleration3d();


        rot = gyro.getRotation3d();

        // boolean tagExists = SmartDashboard.getBoolean("photonvision/Camera_Module_v1/hasTarget", false);
        
        Translation2d pos;

        // var result = camera.getAllUnreadResults();
        // if (result.hasTargets()) {
        //     PhotonTrackedTarget target = result.getBestTarget();
        //     Transform3d pos3d = target.getBestCameraToTarget();
        //     pos = new Translation2d(pos3d.getX(), pos3d.getY());
        // } else {
            pos = lastPose2d.getTranslation();
        // }

        // results.
        // if (!results.isEmpty()) {

        //     double totalX = 0;
        //     double totalY = 0;

        //     // Camera processed a new frame since last
        //     // Get the last one in the list.
        //     var result = results.get(results.size() - 1);
        //     // PhotonTrackedTarget targets = result.getMultiTagResult();

        //     if (result.hasTargets()) {
        //         PhotonTrackedTarget target = result.getBestTarget();
        //         Transform3d pos3d = target.getBestCameraToTarget();
        //         pos = new Translation2d(pos3d.getX(), pos3d.getY());
        //     } else {
        //         pos = lastPose2d.getTranslation();
        //     }
        // }else {
        //     pos = lastPose2d.getTranslation();
        // }


        lastPose2d = new Pose2d(
            pos,
            gyro.getRotation2d() 
        );
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

        sbGyro.setDouble(rot.getX());
        Shuffleboard.update();
        // sbAccleration.set
        
        // SmartDashboard.putNumber("Accel X", accel.getX());
        // SmartDashboard.putNumber("Accel Y", accel.getY());
        // SmartDashboard.putNumber("Accel Z", accel.getZ());


        // SmartDashboard.putNumber("Rot X", rot.getX());
        // SmartDashboard.putNumber("Rot Y", rot.getY());
        // SmartDashboard.putNumber("Rot Z", rot.getZ());
    }

    @Override
    public Status diagnosticStatus() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'diagnosticStatus'");
    }
}