// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMotor;

  public Elevator(TalonFX elevatorTalonFX) {
    elevatorMotor = elevatorTalonFX;

    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    
    var PIDConfigs = new Slot0Configs();
    PIDConfigs.kP = 0;
    PIDConfigs.kI = 0;
    PIDConfigs.kD = 0;
    elevatorMotor.getConfigurator().apply(PIDConfigs);
  }

  //PID methods

  public void PIDPosition(double position) {
    var request = new PositionVoltage(position);
    elevatorMotor.setControl(request);
  }

  public void PIDLevel1() {
    PIDPosition(ElevatorConstants.LEVEL_1);
  }

  public void PIDLevel2() {
    PIDPosition(ElevatorConstants.LEVEL_2);
  }

  public void elevatorUp() {
    elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED_UP);
  }

  public void elevatorDown() {
    elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED_UP);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
