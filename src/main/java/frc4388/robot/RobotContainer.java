/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

// Drive Systems
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.utility.controller.XboxController;
import frc4388.utility.controller.DeadbandedXboxController;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.robot.Constants.SwerveDriveConstants.AutoConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;

// Autos
import frc4388.utility.controller.VirtualController;
import frc4388.robot.commands.GotoLastApril;
import frc4388.robot.commands.LidarAlign;
import frc4388.robot.commands.MoveForTimeCommand;
import frc4388.robot.commands.Swerve.neoJoystickPlayback;
import frc4388.robot.commands.Swerve.neoJoystickRecorder;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// Subsystems
// import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.Vision;
import frc4388.robot.subsystems.Elevator.CoordinationState;
import frc4388.robot.subsystems.Lidar;
import frc4388.robot.subsystems.Elevator;
// import frc4388.robot.subsystems.Endeffector;
import frc4388.robot.subsystems.SwerveDrive;

// Utilites
import frc4388.utility.DeferredBlock;
import frc4388.utility.ReefPositionHelper;
import frc4388.utility.Subsystem;
import frc4388.utility.ReefPositionHelper.Side;
import frc4388.utility.configurable.ConfigurableString;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    public final RobotMap m_robotMap = new RobotMap();
    
    /* Subsystems */
    // private final LED m_robotLED = new LED();
    public final Vision m_vision = new Vision(m_robotMap.leftCamera, m_robotMap.rightCamera);
    public final Lidar m_lidar = new Lidar();
    public final Elevator m_robotElevator= new Elevator(m_robotMap.elevator, m_robotMap.endeffector, m_robotMap.basinLimitSwitch, m_robotMap.endefectorLimitSwitch);
    public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain, m_vision);
    // public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain);


    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);    
    private final DeadbandedXboxController m_autoRecorderXbox = new DeadbandedXboxController(OIConstants.XBOX_PROGRAMMER_ID);

    /* Virtual Controllers */
    private final VirtualController m_virtualDriver = new VirtualController(0);
    private final VirtualController m_virtualOperator = new VirtualController(1);

    // public List<Subsystem> subsystems = new ArrayList<>();

    // ! Teleop Commands
    public void stop() {
        new InstantCommand(()->{}, m_robotSwerveDrive).schedule();;;;;
        m_robotSwerveDrive.stopModules();
    }

    // ! /*  Autos */
    private String lastAutoName = "defualt.auto";
    private final SendableChooser<Command> autoChooser;
    private ConfigurableString autoplaybackName = new ConfigurableString("Auto Playback Name", lastAutoName);
    private neoJoystickPlayback autoPlayback = new neoJoystickPlayback(m_robotSwerveDrive, 
    () -> autoplaybackName.get(), // lastAutoName
           new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
           true, false);
    private Command AutoGotoPosition = new GotoLastApril(m_robotSwerveDrive, m_vision);
    private Command AprilLidarAlign = new SequentialCommandGroup(
        new GotoLastApril(m_robotSwerveDrive, m_vision),
        new InstantCommand(() -> System.out.println("Soup")),
        new WaitCommand(1),
        new LidarAlign(m_robotSwerveDrive, m_lidar)
    );
    private Command AprilLidarLeft = new SequentialCommandGroup(
        AutoGotoPosition.asProxy(),
        new LidarAlign(m_robotSwerveDrive, m_lidar)
    );

    private Command AprilLidarRight = new SequentialCommandGroup(
        AutoGotoPosition.asProxy(),
        new InstantCommand(() -> System.out.println("Soup")),
        new WaitCommand(1),
        new LidarAlign(m_robotSwerveDrive, m_lidar)//,
        // new MoveForTimeCommand(m_robotSwerveDrive, 
        // new Translation2d(0, 0.5), new Translation2d(), 1000, true)
    );

    private Command placeCoral = new SequentialCommandGroup(
        new InstantCommand(() -> m_robotSwerveDrive.stopModules()),
        new InstantCommand(() -> System.out.println("Placing Some Coral")),
        new WaitCommand(3),
        new InstantCommand(() -> System.out.println("Done placing Some Coral"))
    );

    private Command aprilAlign = new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("Aligning...")),
        new WaitCommand(1)
    );

    private Command grabCoral = new SequentialCommandGroup(
        new InstantCommand(() -> System.out.println("Yoinking some coral...")),
        new WaitCommand(2),
        new InstantCommand(() -> System.out.println("Done yoinking some coral..."))
    );
    
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        
        NamedCommands.registerCommand("AutoGotoPosition", AutoGotoPosition);
        NamedCommands.registerCommand("april-allign", AprilLidarAlign);
        NamedCommands.registerCommand("place-coral", placeCoral);
        NamedCommands.registerCommand("grab-coral", grabCoral);

        configureButtonBindings();        
        configureVirtualButtonBindings();
        new DeferredBlock(() -> m_robotSwerveDrive.resetGyro());
        DriverStation.silenceJoystickConnectionWarning(true);
        // CameraServer.startAutomaticCapture();

        /* Default Commands */
        // ! Swerve Drive Default Command (Regular Rotation)
        // drives the robot with a two-axis input from the driver controller
        m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
            // m_robotSwerveDrive.driveWithInput(getDeadbandedDriverController().getLeft(),
            m_robotSwerveDrive.driveWithInput(new Translation2d(.4, 0),
                                            getDeadbandedDriverController().getRight(),
                                true);
        }, m_robotSwerveDrive)
        .withName("SwerveDrive DefaultCommand"));
        m_robotSwerveDrive.setToSlow();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // this.subsystems.add(m_robotSwerveDrive);
        // this.subsystems.add(m_robotMap.leftFront);
        // this.subsystems.add(m_robotMap.rightFront);
        // this.subsystems.add(m_robotMap.rightBack);
        // this.subsystems.add(m_robotMap.leftBack);

        // ! Swerve Drive One Module Test
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotMap.rightFront.go(getDeadbandedDriverController().getLeft());
        // }

        // ! Swerve Drive Default Command (Orientation Rotation)
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInputOrientation(getDeadbandedDriverController().getLeft(), 
        //                                                  getDeadbandedDriverController().getRightX(), 
        //                                                  getDeadbandedDriverController().getRightY(), 
        //                                                  true);
        // }, m_robotSwerveDrive))
        // .withName("SwerveDrive OrientationCommand"));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInput(
        //                                     getDeadbandedDriverController().getLeft(), 
        //                                     getDeadbandedDriverController().getRight(),
        //                                     true);
        // }, m_robotSwerveDrive));




    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // ? /* Driver Buttons */

        DualJoystickButton(getDeadbandedDriverController(), getVirtualDriverController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.resetGyro()));

        // ! /* Speed */
        new JoystickButton(getDeadbandedDriverController(), XboxController.RIGHT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.shiftUp()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.START_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.activateLuigiMode()));
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.BACK_BUTTON) // final
            .onTrue(new InstantCommand(()  -> m_robotSwerveDrive.deactivateLuigiMode()));

        new JoystickButton(getDeadbandedDriverController(), XboxController.LEFT_BUMPER_BUTTON) // final
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDown()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 270)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftDownRot()));

        new Trigger(() -> getDeadbandedDriverController().getPOV() == 90)
            .onTrue(new InstantCommand(() -> m_robotSwerveDrive.shiftUpRot()));
          
        // ?  /* Operator Buttons */

        new JoystickButton(getDeadbandedDriverController(), XboxController.Y_BUTTON)
            // .onTrue(AutoGotoPosition);
            .onTrue(AprilLidarRight);
        
        new JoystickButton(getDeadbandedDriverController(), XboxController.B_BUTTON)
            .onTrue(new InstantCommand(() -> {}, m_robotSwerveDrive, m_lidar)); 

        
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.A_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotElevator.transitionState(CoordinationState.Waiting), m_robotElevator));
        
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.B_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotElevator.transitionState(CoordinationState.Ready), m_robotElevator));
        
        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.Y_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotElevator.transitionState(CoordinationState.ScoringThree), m_robotElevator));

        DualJoystickButton(getDeadbandedOperatorController(), getVirtualOperatorController(), XboxController.X_BUTTON)
            .onTrue(new InstantCommand(() -> m_robotElevator.transitionState(CoordinationState.ScoringFour), m_robotElevator));
        
            
        // ? /* Programer Buttons (Controller 3)*/

        // * /* Auto Recording */
        new JoystickButton(m_autoRecorderXbox, XboxController.LEFT_BUMPER_BUTTON)
            .whileTrue(new neoJoystickRecorder(m_robotSwerveDrive,
                        new DeadbandedXboxController[]{getDeadbandedDriverController(), getDeadbandedOperatorController()},
                                            () -> autoplaybackName.get()))
            .onFalse(new InstantCommand());
        
        new JoystickButton(m_autoRecorderXbox, XboxController.RIGHT_BUMPER_BUTTON)
            .onTrue(new neoJoystickPlayback(m_robotSwerveDrive,
            () -> autoplaybackName.get(),
            new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
            true, false))
            .onFalse(new InstantCommand());

    }
    
    /**
     * This method is used to replcate {@link Trigger Triggers} for {@link VirtualController Virtual Controllers}. <p/>
     * Please use {@link RobotContainer#DualJoystickButton} in {@link RobotContainer#configureButtonBindings} for standard buttons.
     */
    private void configureVirtualButtonBindings() {

        // ? /* Driver Buttons */
        
        /* Notice: the following buttons have not been replicated
         * Swerve Drive Slow and Fast mode Gear Shifts : Fast mode is known to cause drift, so we disable that feature in Autoplayback
         * Swerve Drive Rotation Gear Shifts           : Same reason as Slow and Fast mode.
         * Auto Recording controls                     : We don't want an Null Ouroboros for an auto.
         */

        // ? /* Operator Buttons */

        /* Notice: the following buttons have not been replicated
         * Override Intake Position Encoder : It's an emergancy overide, for when the position of intake when the robot boots, the intake is not inside the robot.
         *                                    We don't need it in an auto.
         * Climbing controls                : We don't need to climb in auto.
         */
        
         // ? Notice: the Programer Buttons are not to be replicated because they are designed for debuging the robot, and do not need to be replicated in auto.

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        //return autoPlayback;
        //return new GotoPositionCommand(m_robotSwerveDrive, m_vision)
        //return autoChooser.getSelected();
	// try{
	//     // Load the path you want to follow using its name in the GUI
	    return new PathPlannerAuto("Red Center Cage");
	// } catch (Exception e) {
	//     DriverStation.reportError("Path planner error: " + e.getMessage(), e.getStackTrace());
	//     return Commands.none();
	// }
	// zach told me to do the below comment
	//return new GotoPositionCommand(m_robotSwerveDrive, m_vision);
      //  return new GotoPositionCommand(m_robotSwerveDrive, m_vision, AutoConstants.targetpos);
    }



    /**
     * A button binding for two controllers, preferably an {@link DeadbandedXboxController Xbox Controller} and {@link VirtualController Virtual Xbox Controller}
     * @param joystickA A controller
     * @param joystickB A controller
     * @param buttonNumber The button to bind to
     */
    public Trigger DualJoystickButton(GenericHID joystickA, GenericHID joystickB, int buttonNumber) {
        return new Trigger(() -> (joystickA.getRawButton(buttonNumber) || joystickB.getRawButton(buttonNumber)));
    }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    public VirtualController getVirtualDriverController() {
        return m_virtualDriver;
    }

    public VirtualController getVirtualOperatorController() {
        return m_virtualOperator;
    }
}
