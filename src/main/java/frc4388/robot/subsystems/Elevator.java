// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMotor;
  private TalonFX endefectorMotor;

  private DigitalInput basinBeamBreak;
  private DigitalInput endefectorLimitSwitch;

  public enum CoordinationState {
    Waiting, // for coral into the though
    WatingBeamTriped, //once the beam break trips
    Ready, // Has coral in endefector
    PrimedThree, // Arm and elevator Waiting to score in the level 3 position
    ScoringThree, // Arm and elevator in the level three position
    PrimedFour, // Arm and elevator Waiting to score in the level 4 position
    ScoringFour, // Arm and elevator in the level four position
    BallRemoverL2Primed, // Arm and elevator ready to remove the ball in the level 2 reef.
    BallRemoverL2Go, // Arm and elevator removing the ball in the level 2 reef.
    BallRemoverL3Primed, // Arm and elevator ready to remove the ball in the level 3 reef.
    BallRemoverL3Go, // Arm and elevator removing the ball in the level 3 reef.
  }

  private CoordinationState currentState;

  public Elevator(TalonFX elevatorTalonFX, TalonFX endefectorTalonFX, DigitalInput basinLimitSwitch, DigitalInput endefectorLimitSwitch) {
    elevatorMotor = elevatorTalonFX;
    endefectorMotor = endefectorTalonFX;

    this.basinBeamBreak = basinLimitSwitch;
    this.endefectorLimitSwitch = endefectorLimitSwitch;

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    endefectorMotor.setNeutralMode(NeutralModeValue.Brake);
    
    elevatorMotor.getConfigurator().apply(ElevatorConstants.ELEVATOR_PID);
    endefectorMotor.getConfigurator().apply(ElevatorConstants.ENDEFECTOR_PID);
    currentState = CoordinationState.Ready;
  }

  //PID methods

  private void PIDPosition(TalonFX motor, double position) {
    var request = new PositionDutyCycle(position);
    motor.setControl(request);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public void endefectorStop() {
    endefectorMotor.set(0);
  }

  public void transitionState(CoordinationState state) {
    currentState = state;
    switch (currentState) {
      case Waiting: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case WatingBeamTriped: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_BEAM_BREAK_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        armShuffle();
        break;
      }

      case Ready: {
        PIDPosition(elevatorMotor, ElevatorConstants.GROUND_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case ScoringThree: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFECTOR);
        break;
      }

      case ScoringFour: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFECTOR);
        break;
      }
    }

  }

  // public void driveElevatorStick(Translation2d stick) {
  //   if (stick.getNorm() > 0.05) {
  //     elevatorMotor.set(stick.getY());
  //   }
  // }

  private void periodicWaiting() {
    if (basinBeamBreak.get()) transitionState(CoordinationState.WatingBeamTriped);
  }

  private void periodicWaitingTripped() {
    if (basinBeamBreak.get()) transitionState(CoordinationState.Ready);
  }
  
  private void periodicScoring() {
    if (!endefectorLimitSwitch.get()) transitionState(CoordinationState.Waiting);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Basin", basinBeamBreak.get() ? 1 : 0);
    SmartDashboard.putNumber("endefector", endefectorLimitSwitch.get() ? 1 : 0);

    // if (currentState == CoordinationState.Waiting) {
    //   periodicWaiting();
    // } else if (currentState == CoordinationState.WatingBeamTriped) {
    //   periodicWaitingTripped();
    // }
    // } else if (currentState == CoordinationState.ScoringThree || currentState == CoordinationState.ScoringFour) {
    //   periodicScoring();
    // }
  }

  public void armShuffle(){
    if(!basinBeamBreak.get()){
      //shuffle the coral with the arm until coral hits beam break
    }
  }
}
