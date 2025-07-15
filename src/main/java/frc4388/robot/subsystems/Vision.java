package frc4388.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.FieldConstants;
import frc4388.robot.constants.Constants.VisionConstants;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;

public class Vision extends SubsystemBase implements Queryable {
    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] estimators;

    @AutoLog
    public class VisionState {
        public boolean isTagDetected = false;
        public boolean isTagProcessed = false;
        public List<EstimatedRobotPose> poses = new ArrayList<>();
        public double latency = 0;
        public Pose2d lastVisionPose = new Pose2d();
        public Pose2d lastPhysOdomPose = new Pose2d();
    }

    private VisionState state = new VisionState();

    private Field2d field = new Field2d();
    public Vision(PhotonCamera leftCamera, PhotonCamera rightCamera){
        FaultReporter.register(this);
        SmartDashboard.putData(field);

        this.cameras = new PhotonCamera[]{leftCamera, rightCamera};

        PhotonPoseEstimator photonEstimatorLeft = new PhotonPoseEstimator(FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.LEFT_CAMERA_POS);
        PhotonPoseEstimator photonEstimatorRight = new PhotonPoseEstimator(FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.RIGHT_CAMERA_POS);

        photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.estimators = new PhotonPoseEstimator[]{photonEstimatorLeft, photonEstimatorRight};
    }

    @Override
    public void periodic() {
        update();
        field.setRobotPose(getPose2d());
    }

    // private Instant lastVisionTime = null;


    private void update() {
        state.isTagProcessed = false;
        state.isTagDetected = false;

        // Instant now = Instant.now();

        // int cams = 0;

        double latency = 0;

        // Pose2d pose = null;
        state.poses.clear();

        for(int i = 0; i < cameras.length; i++){
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];

            var results = camera.getAllUnreadResults();

            // If there are no more updates from the camera
            if (results.size() == 0) 
                continue;

            
            var result = results.get(results.size()-1);
            latency += result.getTimestampSeconds();

            state.isTagDetected = state.isTagDetected | result.hasTargets();

            // If there are no tags
            if(!result.hasTargets())
                continue;

            Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose(result, estimator);

            // If the tag was failed to be processed
            if(estimatedRobotPose.isEmpty())
                continue;
            
            state.poses.add(estimatedRobotPose.get());

            // if(pose == null)
            //     pose = estimatedRobotPose.get().estimatedPose.toPose2d();
            // else
            //     pose = pose.interpolate(pose, 0.5);
            // X += pose.getX();
            // Y += pose.getY();

            // if(X > 6)

            // Yaw += (pose.getRotation().getDegrees() + 180) % 360;
            // cams++;

            state.isTagProcessed = true;
        
            
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
        // updateEstimationStdDevs(visionEst, change.getTargets(), estimator);

        return visionEst;
    }


    // /**
    //  * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
    //  * deviations based on number of tags, estimation strategy, and distance from the tags.
    //  *
    //  * @param estimatedPose The estimated pose to guess standard deviations for.
    //  * @param targets All targets in this camera frame
    //  */
    // private void updateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, 
    //         List<PhotonTrackedTarget> targets,
    //         PhotonPoseEstimator estimator) {
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         curStdDevs = VisionConstants.kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = VisionConstants.kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
                
    //             double distance = tagPose
    //             .get()
    //             .toPose2d()
    //             .getTranslation()
    //             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                
    //             if (distance < VisionConstants.MIN_ESTIMATION_DISTANCE) {
    //                 numTags++;
    //                 avgDist += distance;
    //             }
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             curStdDevs = VisionConstants.kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             curStdDevs = estStdDevs;
    //         }
    //     }
    // }





    public void setLastOdomPose(Optional<Pose2d> pose){
        if(pose.isPresent())
            state.lastPhysOdomPose = pose.get();
    }

    // public double getLastOdomSpeed(){
    //     return lastOdomSpeed;
    // }

    public Pose2d getPose2d() {
        if(state.lastPhysOdomPose != null)
            return state.lastPhysOdomPose;

        // if(lastVisionPose != null)
        //     return lastVisionPose;
        return new Pose2d();

    }

    public static double getTime() {
        return Utils.getCurrentTimeSeconds();
    }

    public boolean isTag(){
        return state.isTagDetected && state.isTagProcessed;
    }


    public void addVisionMeasurement( SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain){
        for(EstimatedRobotPose pose : state.poses){
            drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds));
        }
    }








    @Override
    public String getName() {
        return "Vision";
    }

    @Override
    public Status diagnosticStatus() {
        return new Status();
    }
    
}
