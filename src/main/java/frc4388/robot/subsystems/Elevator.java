// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMotor;
  private TalonFX endefectorMotor;

  private DigitalInput basinLimitSwitch;
  private DigitalInput endefectorLimitSwitch;

  public enum CordinationState {
    Waiting, // for coral into the though 
    Ready, // Has coral in enefector
    ScoringThree, // Arm and elevator in the level three position
    ScoringFour // Arm and elevator in the level four position

  }

  private CordinationState currentState;

  public Elevator(TalonFX elevatorTalonFX, TalonFX endefectorTalonFX, DigitalInput basinLimitSwitch, DigitalInput endefectorLimitSwitch) {
    elevatorMotor = elevatorTalonFX;
    endefectorMotor = endefectorTalonFX;

    this.basinLimitSwitch = basinLimitSwitch;
    this.endefectorLimitSwitch = endefectorLimitSwitch;

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    endefectorMotor.setNeutralMode(NeutralModeValue.Brake);
    
    elevatorMotor.getConfigurator().apply(ElevatorConstants.ELEVATOR_PID);
    endefectorMotor.getConfigurator().apply(ElevatorConstants.ENDEFECTOR_PID);
    currentState = CordinationState.Ready;
  }

  //PID methods

  private void PIDPosition(TalonFX motor, double position) {
    var request = new PositionVoltage(position);
    elevatorMotor.setControl(request);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public void endefectorStop() {
    endefectorMotor.set(0);
  }
  
  public void transitionWaiting() {
    PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
    PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
    currentState = CordinationState.Waiting;
  }

  private void periodicWaiting() {
    if (basinLimitSwitch.get()) transitionReady();
  }

  public void transitionReady() {
    PIDPosition(elevatorMotor, ElevatorConstants.GROUND_POSITION_ELEVATOR);
    PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
    currentState = CordinationState.Ready;
  }

  public void transitionScoringThree() {
    PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR);
    PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFECTOR);
    currentState = CordinationState.ScoringThree;
  }
  
  private void periodicScoring() {
    if (!endefectorLimitSwitch.get()) transitionWaiting();
  }

  public void transitionScoringFour() {
    PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR);
    PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFECTOR);
    currentState = CordinationState.ScoringFour;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (currentState == CordinationState.Waiting) {
      periodicWaiting();
    } else if (currentState == CordinationState.ScoringThree || currentState == CordinationState.ScoringFour) {
      periodicScoring();
    }
  }
}
