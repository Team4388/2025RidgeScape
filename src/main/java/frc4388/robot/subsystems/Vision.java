package frc4388.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.Constants.VisionConstants;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;

public class Vision extends Subsystem {

    private PhotonCamera camera;

    private boolean isTag = false;
    private Pose2d lastVisionPose = new Pose2d();
    private Pose2d lastPhysOdomPose = new Pose2d();

    private Matrix<N3, N1> curStdDevs;
    private final PhotonPoseEstimator photonEstimator;

    private Field2d field = new Field2d();

    
    ShuffleboardLayout subsystemLayout = Shuffleboard.getTab("Subsystems")
    .getLayout(getSubsystemName(), BuiltInLayouts.kList)
    .withSize(2, 2);

    GenericEntry sbTag = subsystemLayout
    .add("Tag Detected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    
    GenericEntry sbCamConnected = subsystemLayout
    .add("Camera Connnected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    
    public Vision(PhotonCamera camera){
        this.camera = camera;
        SmartDashboard.putData(field);

        photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.CAMERA_POS);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();
        // var results = camera.getAllUnreadResults();
        // if (results.size() == 0) return;
        // var result = results.get(results.size()-1);
        isTag = result.hasTargets();
        
        // Optional<MultiTargetPNPResult> multitag = result.getMultiTagResult();

        // if (multitag.isEmpty()) {
        //     Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
        // }else if()

        
        // sbTag.setBoolean(isTag);
        // field.setRobotPose(getPose2d());

        // sbCamConnected.setBoolean(camera);

        // System.out.println(isTag);

        if(!isTag){
            sbTag.setBoolean(isTag);
            field.setRobotPose(getPose2d());
            return;
        }

        var EstimatedRobotPose = getEstimatedGlobalPose();

        // In case the pose estimator fails to estimate the pose, fallback to physical odometry.
        if(EstimatedRobotPose.isEmpty()){
            isTag = false;
            sbTag.setBoolean(isTag);
            field.setRobotPose(getPose2d());
            return;
        }

        lastVisionPose = EstimatedRobotPose.get().estimatedPose.toPose2d();
        lastVisionPose.rotateBy(Rotation2d.k180deg);
        // lastVisionPose = new Pose2d(
        //     lastVisionPose.getTranslation(),
        //     lastPhysOdomPose.getRotation()
        // );

        
        
        field.setRobotPose(getPose2d());
    }




    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            // if (Robot.isSimulation()) {
            //     visionEst.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             });
            // }
        }
        return visionEst;
    }







    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }









    public void setLastOdomPose(Optional<Pose2d> pose){
        if(pose.isPresent())
            lastPhysOdomPose = pose.get();
    }

    public Pose2d getPose2d() {
        if(isTag)
            return lastVisionPose;
        else
            return lastPhysOdomPose;
    }

    public static double getTime() {
        return Utils.getCurrentTimeSeconds();
    }

    public boolean isTag(){
        return isTag;
    }











    @Override
    public String getSubsystemName() {
        return "Vision";
    }

//   GenericEntry sbShiftState = subsystemLayout
//   .add("Shift State", 0)
//   .withWidget(BuiltInWidgets.kNumberBar)
//   .getEntry();


    @Override
    public void queryStatus() {
        sbTag.setBoolean(isTag);
        sbCamConnected.setBoolean(camera.isConnected());
        // field.setRobotPose(getPose2d());
    }

    @Override
    public Status diagnosticStatus() {
        Status status = new Status();

        return status;
    }
    
}
