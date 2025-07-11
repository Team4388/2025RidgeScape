/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.constants.Constants.ElevatorConstants;
import frc4388.robot.constants.Constants.LiDARConstants;
import frc4388.robot.constants.Constants.VisionConstants;
import frc4388.robot.constants.DriveConstants;
import frc4388.robot.subsystems.Lidar;

/**
 * Defines and holds all I/O objects on the Roborio. This is useful for unit
 * testing and modularization.
 */
public class RobotMap {
    // private Pigeon2 m_pigeon2 = new Pigeon2(SwerveDriveConstants.IDs.DRIVE_PIGEON.id);
    // public RobotGyro gyro = new RobotGyro(m_pigeon2);

    public final PhotonCamera leftCamera = new PhotonCamera(VisionConstants.LEFT_CAMERA_NAME);
    public final PhotonCamera rightCamera = new PhotonCamera(VisionConstants.RIGHT_CAMERA_NAME);

    public final Lidar reefLidar = new Lidar(LiDARConstants.REEF_LIDAR_DIO_CHANNEL, "Reef");
    public final Lidar reverseLidar = new Lidar(LiDARConstants.REVERSE_LIDAR_DIO_CHANNEL, "Reverse");

    
    public RobotMap() {
        configureDriveMotorControllers();
    }

    /* LED Subsystem */
    // public final Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);
    
    /* Swreve Drive Subsystem */
    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDrivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder> (TalonFX::new, TalonFX::new, CANcoder::new, 
        DriveConstants.DrivetrainConstants, 
        DriveConstants.FRONT_LEFT, DriveConstants.FRONT_RIGHT,
        DriveConstants.BACK_LEFT, DriveConstants.BACK_RIGHT
    );

    /* Elevator Subsystem */
    public final TalonFX elevator = new TalonFX(ElevatorConstants.ELEVATOR_ID.id);
    public final TalonFX endeffector = new TalonFX(ElevatorConstants.ENDEFFECTOR_ID.id);
    

    public final DigitalInput basinLimitSwitch = new DigitalInput(ElevatorConstants.BASIN_LIMIT_SWITCH);
    public final DigitalInput endeffectorLimitSwitch = new DigitalInput(ElevatorConstants.ENDEFFECTOR_LIMIT_SWITCH);
    public final DigitalInput IRIntakeBeam = new DigitalInput(ElevatorConstants.INTAKE_LIMIT_SWITCH);

    void configureDriveMotorControllers() {
        // endeffector.saf
    }


    public class RobotMapSim {
        public PhotonCameraSim leftCamera;
        public PhotonCameraSim rightCamera;
    }

    public RobotMapSim configureSim() {
        RobotMapSim sim = new RobotMapSim();

        // The simulated camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        sim.leftCamera = new PhotonCameraSim(leftCamera, cameraProp);
        sim.rightCamera = new PhotonCameraSim(rightCamera, cameraProp);

        
        sim.leftCamera.enableRawStream(true);
        sim.leftCamera.enableProcessedStream(true);
        sim.leftCamera.enableDrawWireframe(true);


        sim.rightCamera.enableRawStream(true);
        sim.rightCamera.enableProcessedStream(true);
        sim.rightCamera.enableDrawWireframe(true);

        return sim;

    }
   
}