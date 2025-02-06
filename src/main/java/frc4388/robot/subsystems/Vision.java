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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.robot.Constants.FieldConstants;
import frc4388.robot.Constants.VisionConstants;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;

public class Vision extends Subsystem {

    // private PhotonCamera leftCamera;
    // private PhotonCamera rightCamera;

    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] estimators;

    private boolean isTagDetected = false;
    private boolean isTagProcessed = false;
    private Pose2d lastVisionPose = new Pose2d();
    private Pose2d lastPhysOdomPose = new Pose2d();

    private Matrix<N3, N1> curStdDevs;
    private final PhotonPoseEstimator photonEstimatorLeft;
    private final PhotonPoseEstimator photonEstimatorRight;

    private Field2d field = new Field2d();

    
    ShuffleboardLayout subsystemLayout = Shuffleboard.getTab("Subsystems")
    .getLayout(getSubsystemName(), BuiltInLayouts.kList)
    .withSize(2, 2);

    GenericEntry sbTagDetected = subsystemLayout
    .add("Tag Detected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    GenericEntry sbTagProcessed = subsystemLayout
    .add("Tag Processed", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    
    GenericEntry sbLeftCamConnected = subsystemLayout
    .add("Left Camera Connnected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    GenericEntry sbRightCamConnected = subsystemLayout
    .add("Right Camera Connnected", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();
    
    public Vision(PhotonCamera leftCamera, PhotonCamera rightCamera){
        SmartDashboard.putData(field);

        this.cameras = new PhotonCamera[]{leftCamera, rightCamera};

        photonEstimatorLeft = new PhotonPoseEstimator(FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.LEFT_CAMERA_POS);
        photonEstimatorRight = new PhotonPoseEstimator(FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.RIGHT_CAMERA_POS);

        photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.estimators = new PhotonPoseEstimator[]{photonEstimatorLeft, photonEstimatorRight};
    }

    @Override
    public void periodic() {
        update();
        field.setRobotPose(getPose2d());
    }


    private void update() {
        isTagProcessed = false;
        isTagDetected = false;


        int cams = 0;

        double X = 0;
        double Y = 0;
        double Yaw = 0;

        for(int i = 0; i < cameras.length; i++){
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];

            var results = camera.getAllUnreadResults();

            // If there are no more updates from the camera
            if (results.size() == 0) 
                continue;

            
            var result = results.get(results.size()-1);

            isTagDetected = isTagDetected | result.hasTargets();

            // If there are no tags
            if(!result.hasTargets())
                continue;

            Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose(result, estimator);

            // If the tag was failed to be processed
            if(estimatedRobotPose.isEmpty())
                continue;

            Pose2d pose = estimatedRobotPose.get().estimatedPose.toPose2d();
            X += pose.getX();
            Y += pose.getY();
            Yaw += pose.getRotation().getDegrees();
            cams++;
            isTagProcessed = true;
        
            
        }

        if(isTagProcessed){
            lastVisionPose = new Pose2d(X/cams, Y/cams, Rotation2d.fromDegrees(Yaw/cams));
        }
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
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult change, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        var targets = change.getTargets();
        for(int i = targets.size()-1; i >= 0; i--){
            Transform3d pos = targets.get(i).getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(pos.getX(),2) + Math.pow(pos.getY(),2) + Math.pow(pos.getZ(),2));
            if (distance > VisionConstants.MIN_ESTIMATION_DISTANCE) {
                change.targets.remove(i);
            }
        }

        if(targets.size() <= 0)
            return visionEst; // Will be empty

        visionEst = estimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets(), estimator);

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
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
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
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                
                double distance = tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                
                if (distance < VisionConstants.MIN_ESTIMATION_DISTANCE) {
                    numTags++;
                    avgDist += distance;
                }
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
        if(isTagDetected && isTagProcessed)
            return lastVisionPose;
        else
            return lastPhysOdomPose;
    }

    public static double getTime() {
        return Utils.getCurrentTimeSeconds();
    }

    public boolean isTag(){
        return isTagDetected && isTagProcessed;
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
        sbTagDetected.setBoolean(isTagDetected);
        sbTagProcessed.setBoolean(isTagProcessed);
        sbLeftCamConnected.setBoolean(cameras[0].isConnected());
        sbRightCamConnected.setBoolean(cameras[1].isConnected());
        // field.setRobotPose(getPose2d());
    }

    @Override
    public Status diagnosticStatus() {
        Status status = new Status();

        return status;
    }
    
}
