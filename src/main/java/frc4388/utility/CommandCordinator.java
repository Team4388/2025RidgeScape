// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.commands.GotoLastApril;
import frc4388.robot.subsystems.Elevator;
import frc4388.robot.subsystems.Lidar;
import frc4388.robot.subsystems.SwerveDrive;
import frc4388.robot.subsystems.Vision;

/** Add your docs here. */
public class CommandCordinator extends Command {
    private SwerveDrive m_robotSwerve;
    private Vision m_robotVision;
    private Lidar m_robotLidar;
    private Elevator m_robotElevator;

    private boolean targetingLeft;
    private boolean targetingLevelFour;

    private boolean isActive;

    private int stage;

    private GotoLastApril AprilHeadedLeft;

    public CommandCordinator(SwerveDrive swerveDrive, Vision vision, Lidar lidar, Elevator elevator) {
        m_robotSwerve = swerveDrive;
        m_robotVision = vision;
        m_robotLidar = lidar;
        m_robotElevator = elevator;

        targetingLeft = false;
        targetingLevelFour = false;

        isActive = false;
        stage = 0;

        AprilHeadedLeft = new GotoLastApril(m_robotSwerve, m_robotVision, true);
    }

    public void setTargetingLeft(boolean value) {
        targetingLeft = value;
    }

    public void setTargetingLevelFour(boolean value) {
        targetingLevelFour = value;
    }

    @Override
    public void initialize() {
        isActive = true;
        stage = 0;
    }

    @Override
    public void execute() {
        
    }
}
