package frc4388.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.utility.UtilityStructs.AutoRecordingControllerFrame;
import frc4388.utility.UtilityStructs.AutoRecordingFrame;
import frc4388.utility.controller.VirtualController;

public class GotoPositionCommand extends Command {
    private boolean isFinished = false;

    SwerveDrive swerveDrive;

    /**
     * Command to drive robot to position.
     * @param SwerveDrive m_robotSwerveDrive
     */
    public GotoPositionCommand(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        

    }

    @Override
    public void end(boolean interrupted) {
       
    }
    
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
