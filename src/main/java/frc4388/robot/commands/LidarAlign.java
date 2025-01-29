// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.Lidar;
import frc4388.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LidarAlign extends Command {
  private SwerveDrive swerveDrive;
  private Lidar lidar;  

  private int currentFinderTick;
  // private int tickFoundPipe;
  private boolean foundReef;
  private boolean headedRight;
  private final boolean constructedHeadedRight;

  /** Creates a new LidarAlign. */
  public LidarAlign(SwerveDrive swerveDrive, Lidar lidar, boolean headedRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    constructedHeadedRight = headedRight;

    this.swerveDrive = swerveDrive;
    this.lidar = lidar;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentFinderTick = 0;
    this.foundReef = false;
    this.headedRight = constructedHeadedRight;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.05; // TODO: find good speed for this
    double relAngle = Math.round(swerveDrive.getGyroAngle() / 60.d) * 60; // Relative driving to the side of the reef
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
