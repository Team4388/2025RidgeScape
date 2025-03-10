package frc4388.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.Constants.AutoConstants;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Vision;
import frc4388.utility.Gains;
import frc4388.utility.ReefPositionHelper;
import frc4388.utility.TimesNegativeOne;
import frc4388.utility.ReefPositionHelper.Side;
import frc4388.utility.UtilityStructs.AutoRecordingControllerFrame;
import frc4388.utility.UtilityStructs.AutoRecordingFrame;
import frc4388.utility.controller.VirtualController;

public class GotoLastApril extends Command {
    

    // private Translation2d translation2d= new Translation2d(14.579471999999997,0.24587199999999998);
    // private Translation2d translation2d= new Translation2d(16.579342-0.15,5.547867999999999);

    private PID xPID = new PID(AutoConstants.XY_GAINS, 0);
    private PID yPID = new PID(AutoConstants.XY_GAINS, 0);
    private PID rotPID = new PID(AutoConstants.ROT_GAINS, 0);
    private Pose2d targetpos;

    SwerveDrive swerveDrive;
    Vision vision;
    double distance;
    Side side;

    /**
     * Command to drive robot to position.
     * @param SwerveDrive m_robotSwerveDrive
     */

    public GotoLastApril(SwerveDrive swerveDrive, Vision vision, double distance, Side side) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.distance = distance;
        this.side = side;
        // addRequirements(swerveDrive);
    }


    public static double tagRelativeXError = -1;
    private static void setTagRelativeXError(double val){
        tagRelativeXError = val;
    }

    Alliance alliance = null;

    @Override
    public void initialize() {
        xPID.initialize();
        yPID.initialize();
        this.targetpos = ReefPositionHelper.getNearestPosition(this.vision.getPose2d(), side, 
        Units.inchesToMeters(AutoConstants.X_OFFSET_TRIM.get()), 
        distance + Units.inchesToMeters(AutoConstants.Y_OFFSET_TRIM.get()));
        Optional<Alliance> a = DriverStation.getAlliance();
        if(!a.isEmpty())
            alliance = a.get();
    }
    
    double xerr;
    double yerr;
    double roterr;

    @Override
    public void execute() {
        xerr = TimesNegativeOne.invert(targetpos.getX() - vision.getPose2d().getX(), TimesNegativeOne.XAxis);
        yerr = TimesNegativeOne.invert(targetpos.getY() - vision.getPose2d().getY(), !TimesNegativeOne.YAxis);
        // xerr = targetpos.getX() - vision.getPose2d().getX();
        // yerr = targetpos.getX() - vision.getPose2d().getY();

        // roterr = TimesNegativeOne.invert(targetpos.getRotation().getDegrees() - vision.getPose2d().getRotation().getDegrees(), TimesNegativeOne.isRed);

        roterr = ((targetpos.getRotation().getDegrees() - vision.getPose2d().getRotation().getDegrees()));

        boolean invert = Math.abs(roterr) > 180;

        if(roterr > 180){
            roterr -= 360;
        }else if(roterr < -180){
            roterr += 360;
        }

        SmartDashboard.putNumber("Rotational PID target", targetpos.getRotation().getDegrees());
        SmartDashboard.putNumber("Rotational PID position", vision.getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Rotational PID error", roterr);

        // SmartDashboard.putNumber("PID X Error", xerr);
        // SmartDashboard.putNumber("PID Y Error", yerr);

        double xoutput = xPID.update(xerr);
        double youtput = yPID.update(yerr);
        double rotoutput = rotPID.update(roterr);

        xoutput *= Math.abs(xerr) < AutoConstants.XY_TOLERANCE ? 0 : 1;
        youtput *= Math.abs(yerr) < AutoConstants.XY_TOLERANCE ? 0 : 1;
        rotoutput *= Math.abs(roterr) < AutoConstants.ROT_TOLERANCE ? 0 : 1;
        


        Translation2d leftStick = new Translation2d(
            Math.max(Math.min(-youtput, 1), -1),
            Math.max(Math.min(-xoutput, 1), -1)
            // 0,0
        );

        Translation2d rightStick = new Translation2d(
            Math.max(Math.min(rotoutput, 1), -1), 
           0
        );



        setTagRelativeXError(
            new Translation2d(xerr, yerr).getAngle()
            .rotateBy(targetpos.getRotation())
            .getCos());

        swerveDrive.driveWithInput(leftStick, rightStick, true);
    }

    @Override
    public final boolean isFinished() {
        boolean finished = (Math.abs(xerr) < AutoConstants.XY_TOLERANCE && Math.abs(yerr) < AutoConstants.XY_TOLERANCE && Math.abs(roterr) < AutoConstants.ROT_TOLERANCE);
        // System.out.println(finished);

        if(finished)
            swerveDrive.softStop();

        return finished;
                // this statement is a boolean in and of itself
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
