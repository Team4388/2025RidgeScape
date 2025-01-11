package frc4388.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Vision;
import frc4388.utility.Gains;
import frc4388.utility.UtilityStructs.AutoRecordingControllerFrame;
import frc4388.utility.UtilityStructs.AutoRecordingFrame;
import frc4388.utility.controller.VirtualController;

public class GotoPositionCommand extends Command {
    

    // private Translation2d translation2d= new Translation2d(14.579471999999997,0.24587199999999998);
    // private Translation2d translation2d= new Translation2d(16.579342-0.15,5.547867999999999);
    private Pose2d targetpos = new Pose2d(new Translation2d(0.3,0), new Rotation2d());
    static Gains xygains = new Gains(3,0,0);
    static Gains rotgains = new Gains(0.1,0,0.0);
    static double tolerance = 0;

    private PID xPID = new PID(xygains, 0);
    private PID yPID = new PID(xygains, 0);
    private PID rotPID = new PID(rotgains, 0);

    SwerveDrive swerveDrive;
    Vision vision;

    /**
     * Command to drive robot to position.
     * @param SwerveDrive m_robotSwerveDrive
     */

    public GotoPositionCommand(SwerveDrive swerveDrive, Vision vision) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        xPID.initialize();
        yPID.initialize();
    }
    
    @Override
    public void execute() {
        double xerr = targetpos.getX() - vision.getPose2d().getX();
        double yerr = targetpos.getY() - vision.getPose2d().getY();
        double roterr = targetpos.getRotation().getDegrees() - vision.getPose2d().getRotation().getDegrees();

        SmartDashboard.putNumber("PID X Error", xerr);
        SmartDashboard.putNumber("PID Y Error", yerr);

        double xoutput = xPID.update(xerr);
        double youtput = yPID.update(yerr);
        double rotoutput = rotPID.update(roterr);

        Translation2d leftStick = new Translation2d(
            Math.max(Math.min(youtput, 1), -1),
            Math.max(Math.min(xoutput, 1), -1)
            // 0,0
        );

        Translation2d rightStick = new Translation2d(
            Math.max(Math.min(rotoutput, 1), -1), 
           0
        );

        SmartDashboard.putNumber("PID X Output", xoutput);
        SmartDashboard.putNumber("PID Y Output", youtput);
        // SmartDashboard.putNumber("PID Y Output", youtput);

        swerveDrive.driveWithInput(leftStick, rightStick, true);
    }
    // @Override
    // public void end(boolean interrupted) {
       
    // }
    


    // @Override
    // public double getError() {
    //     return e; 
    // }



    // @Override
    // public void runWithOutput(double output) {
    //     // TODO Auto-generated method stub
    //     Translation2d leftStick = new Translation2d(Math.max(Math.min(output, 1), -1),0);
    //     Translation2d rightStick = new Translation2d();
    //     // System.out.println("Output = " + -output);
    //     SmartDashboard.putNumber("PID Output", output);
    //     swerveDrive.driveWithInput(leftStick, rightStick, true);
    // }

















    private class PID {
        protected Gains  gains;
        private   double output    = 0;
        private   double tolerance = 0;

        /** Creates a new PelvicInflammatoryDisease. */
        public PID(double kp, double ki, double kd, double kf, double tolerance) {
            gains          = new Gains(kp, ki, kd, kf, 0);
            this.tolerance = tolerance;
        }

        public PID(Gains gains, double tolerance) {
            this.gains     = gains;
            this.tolerance = tolerance;
        }

        // Called when the command is initially scheduled.
        public final void initialize() {
            output = 0;
        }

        private double prevError, cumError = 0;
        
        // Called every time the scheduler runs while the command is scheduled.
        public double update(double error) {
            cumError += error * .02; // 20 ms
            double delta = error - prevError;

            output = error * gains.kP;
            output += cumError * gains.kI;
            output += delta * gains.kD;
            output += gains.kF;

            return output;
        }

        // // Returns true when the command should end.
        // public final boolean isFinished() {
        //     return Math.abs(getError()) < tolerance;
        // }
    }
}
