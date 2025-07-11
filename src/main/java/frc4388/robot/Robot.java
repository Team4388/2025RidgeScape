/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import java.util.ArrayList;
import java.util.Base64;
import java.util.List;

// Advantagekit
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc4388.robot.constants.BuildConstants;
import frc4388.robot.constants.Constants;
import frc4388.robot.constants.Constants.SimConstants;
import frc4388.utility.DeferredBlock;
import frc4388.utility.DeferredBlockMulti;
import frc4388.utility.Trim;
import frc4388.utility.compute.RobotTime;
import frc4388.utility.status.CanDevice;
import frc4388.utility.status.Status;
import frc4388.utility.status.Subsystem;
import frc4388.utility.status.Status.Report;
import frc4388.utility.status.Status.ReportLevel;
//import frc4388.robot.subsystems.LED;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  Command m_autonomousCommand;

  private RobotTime m_robotTime = RobotTime.getInstance();
  private RobotContainer m_robotContainer;
  //private LED mled = new LED();
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    // Create a shuffleboard update thread, that will periodically update the values on shuffleboard
    new Thread() {
      public void run() {
        try{
        while(!this.isInterrupted() && this.isAlive()){
          Thread.sleep(500);
          for(int i=0;i<Subsystem.subsystems.size(); i++){
            Subsystem.subsystems.get(i).queryStatus();
          }

          // System.out.println("Updated statuses!");
          
        }
        }catch(Exception e){
            e.printStackTrace();
        }
      }
    }.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.doubl
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {   
    m_robotTime.updateTimes();
    // SmartDashboard.putNumber("Time", System.currentTimeMillis());
    
    m_robotContainer.m_robotLED.update();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    m_robotTime.endMatchTime();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    DeferredBlock.execute();
    DeferredBlockMulti.execute();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.stop();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
      m_autonomousCommand.cancel();
      m_autonomousCommand.end(true);
      System.out.println("NOT Null!!");

    } else {
      System.out.println("Null!!");
    }
    m_robotTime.startMatchTime();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  //  m_robotContainer.m_robotMap.rightFront.go(m_robotContainer.getDeadbandedDriverController().getLeft());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopExit() { // the only OTHER mode that teleop can enter into is disabled.
    Trim.dumpAll();
  }

  @Override
  public void testInit() {

    List<String> errors = new ArrayList<>();

    // Subsystems header
    System.out.println(new String(Base64.getDecoder().decode("IOKWl+KWhOKWhOKWluKWl+KWliDilpfilpbilpfiloTiloTilpYgIOKWl+KWhOKWhOKWluKWl+KWliAg4paX4paW4paX4paE4paE4paW4paX4paE4paE4paE4paW4paX4paE4paE4paE4paW4paX4paWICDilpfilpYg4paX4paE4paE4paWCuKWkOKWjCAgIOKWkOKWjCDilpDilozilpDilowg4paQ4paM4paQ4paMICAgIOKWneKWmuKWnuKWmOKWkOKWjCAgICAg4paIICDilpDilowgICDilpDilpvilprilp7ilpzilozilpDilowgICAKIOKWneKWgOKWmuKWluKWkOKWjCDilpDilozilpDilpviloDilprilpYg4pad4paA4paa4paWICDilpDilowgIOKWneKWgOKWmuKWliAg4paIICDilpDilpviloDiloDilpjilpDilowgIOKWkOKWjCDilp3iloDilprilpYK4paX4paE4paE4pae4paY4pad4paa4paE4pae4paY4paQ4paZ4paE4pae4paY4paX4paE4paE4pae4paYICDilpDilowg4paX4paE4paE4pae4paYICDiloggIOKWkOKWmeKWhOKWhOKWluKWkOKWjCAg4paQ4paM4paX4paE4paE4pae4paY")));

    for(int i=0;i< Subsystem.subsystems.size();i++){

      Subsystem subsystem = Subsystem.subsystems.get(i);
      System.out.println("** Subsystem diagnostic report for " + subsystem.getName() + ":");
      Status status = subsystem.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        if(r.reportLevel == ReportLevel.ERROR)
          errors.add(subsystem.getName() + " - " + r.toString());
        subsystem.Log(r.toString());
      }
    }

    
    // CAN header
    System.out.println(new String(Base64.getDecoder().decode("IOKWl+KWhOKWhOKWliDilpfiloTilpYg4paX4paWICDilpfilpYK4paQ4paMICAg4paQ4paMIOKWkOKWjOKWkOKWm+KWmuKWluKWkOKWjArilpDilowgICDilpDilpviloDilpzilozilpDilowg4pad4pac4paMCuKWneKWmuKWhOKWhOKWluKWkOKWjCDilpDilozilpDilowgIOKWkOKWjCh0KQ==")));
    
    CANBus canBus = new CANBus(Constants.CANBUS_NAME);
    
    CANBusStatus canInfo = canBus.getStatus();
    
    System.out.println("CANInfo BusOffCount     - " + canInfo.BusOffCount);
    System.out.println("CANInfo BusUtilization  - " + canInfo.BusUtilization);
    System.out.println("CANInfo RX Errors count - " + canInfo.REC);
    System.out.println("CANInfo TX Errors count - " + canInfo.TEC);
    System.out.println("CANInfo Transmit buffer full count - " + canInfo.TxFullCount);
    // Broken turniary operator
    ReportLevel canReportLevel = canInfo.Status.isOK() ? (canInfo.Status.isWarning() ? ReportLevel.WARNING : ReportLevel.ERROR) : ReportLevel.INFO;
    String canStatus = "CAN " + canReportLevel.name() + " - " + canInfo.Status.getName() + " (" + canInfo.Status.getDescription() + ")";
    if(canReportLevel == ReportLevel.ERROR) {
      errors.add(canStatus);
    }
    System.out.println(canStatus);

    for(int i=0;i<CanDevice.devices.size();i++){

      CanDevice device = CanDevice.devices.get(i);
      System.out.println("** CAN diagnostic report for " + device.name + ":");
      Status status = device.diagnosticStatus();

      for(int a=0;a<status.reports.size();a++){
        Report r = status.reports.get(a);
        if(r.reportLevel == ReportLevel.ERROR)
          errors.add(device.getName() + " - " + r.toString());
        device.Log(r.toString());
      }
    }

    // System.out.println("Found CAN devices: " + new DeviceFinder().Find());
    
    if(errors.size() > 0) {
      // Errors header
      System.out.println(new String(Base64.getDecoder().decode("4paX4paE4paE4paE4paW4paX4paE4paE4paWIOKWl+KWhOKWhOKWliAg4paX4paE4paWIOKWl+KWhOKWhOKWliAg4paX4paE4paE4paWCuKWkOKWjCAgIOKWkOKWjCDilpDilozilpDilowg4paQ4paM4paQ4paMIOKWkOKWjOKWkOKWjCDilpDilozilpDilowgICAK4paQ4pab4paA4paA4paY4paQ4pab4paA4paa4paW4paQ4pab4paA4paa4paW4paQ4paMIOKWkOKWjOKWkOKWm+KWgOKWmuKWliDilp3iloDilprilpYK4paQ4paZ4paE4paE4paW4paQ4paMIOKWkOKWjOKWkOKWjCDilpDilozilp3ilpriloTilp7ilpjilpDilowg4paQ4paM4paX4paE4paE4pae4paY")));
      for(int i=0;i<errors.size(); i++){
        System.out.println(errors.get(i));
      }
    }

  }







  // VisionSystemSim visionSim;
  // RobotMapSim robotMapSim;

  // @Override
  // public void simulationInit() {
  //   visionSim = new VisionSystemSim("main");
  //   robotMapSim = m_robotContainer.m_robotMap.configureSim();


  //   // A 0.5 x 0.25 meter rectangular target
  //   TargetModel targetModel = new TargetModel(0.5, 0.25);
  //   // The pose of where the target is on the field.
  //   // Its rotation determines where "forward" or the target x-axis points.
  //   // Let's say this target is flat against the far wall center, facing the blue driver stations.
  //   Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
  //   // The given target model at the given pose
  //   VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

  //   // Add this vision target to the vision system simulation to make it visible
  //   visionSim.addVisionTargets(visionTarget);

  //   // The layout of AprilTags which we want to add to the vision system
  //   AprilTagFieldLayout tagLayout = Constants.FieldConstants.kTagLayout;

  //   visionSim.addAprilTags(tagLayout);


  //   visionSim.addCamera(robotMapSim.leftCamera, Constants.VisionConstants.LEFT_CAMERA_POS);
  //   visionSim.addCamera(robotMapSim.rightCamera, Constants.VisionConstants.RIGHT_CAMERA_POS);

  //   SmartDashboard.putData(visionSim.getDebugField());
  // }


  // @Override
  // public void simulationPeriodic() {
  //   m_robotContainer.m_robotSwerveDrive.updateSim(RobotController.getBatteryVoltage());
  //   visionSim.update(m_robotContainer.m_robotSwerveDrive.getPose2d());

  //   // m_robotContainer.m_robotSwerveDrive.
  // }



// Start AdvantageKit logging / replay and record metadata
// Refrence: https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/template_projects/sources/skeleton/src/main/java/frc/robot/Robot.java
  public void startLogging() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (SimConstants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }
}