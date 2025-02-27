// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import java.time.Instant;

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
import frc4388.robot.Constants.SwerveDriveConstants.AutoConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMotor;
  private TalonFX endeffectorMotor;

  private long wait = 0;
  private long maxWait = 1000;

  private double elevatorRefrence = 0;
  private double endeffectorRefrence = 0;

  private boolean elevatorManualStop = true;
  private boolean endefectorManualStop = true;

  private DigitalInput basinBeamBreak;
  private DigitalInput endeffectorLimitSwitch;

  public enum CoordinationState {
    Waiting, // for coral into the though
    WatingBeamTripped, //once the beam break trips
    Ready, // Has coral in endefector
    Hovering, // Has coral in endefector
    L2Score,
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

  public Elevator(TalonFX elevatorTalonFX, TalonFX endeffectorTalonFX, DigitalInput basinLimitSwitch, DigitalInput endeffectorLimitSwitch) {
    elevatorMotor = elevatorTalonFX;
    endeffectorMotor = endeffectorTalonFX;

    this.basinBeamBreak = basinLimitSwitch;
    this.endeffectorLimitSwitch = endeffectorLimitSwitch;

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    endeffectorMotor.setNeutralMode(NeutralModeValue.Brake);
    
    elevatorMotor.getConfigurator().apply(ElevatorConstants.ELEVATOR_PID);
    endeffectorMotor.getConfigurator().apply(ElevatorConstants.ENDEFFECTOR_PID);
    currentState = CoordinationState.Ready;
  }

  //PID methods

  private void PIDPosition(TalonFX motor, double position) {
    if (motor == elevatorMotor) elevatorRefrence = position;
    else endeffectorRefrence = position;

    var request = new PositionDutyCycle(position);
    motor.setControl(request);
  }

  public void elevatorStop() {
    elevatorMotor.set(0);
  }

  public void endeffectorStop() {
    endeffectorMotor.set(0);
  }

  public void transitionState(CoordinationState state) {
    currentState = state;
    switch (currentState) {
      case Waiting: {
        wait = System.currentTimeMillis() + maxWait;
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        break;
      }

      case WatingBeamTripped: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_BEAM_BREAK_ELEVATOR);
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        break;
      }

      case Ready: {
        PIDPosition(elevatorMotor, ElevatorConstants.GROUND_POSITION_ELEVATOR);
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        break;
      }

      case Hovering: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        break;
      }

      case L2Score: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.L2_SCORE_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }
      
      case PrimedFour: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFFECTOR);
        break;
      }

      case ScoringFour: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.SCORING_FOUR_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case PrimedThree: {
        PIDPosition(elevatorMotor, ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.PRIMED_THREE_ENDEFFECTOR);
        break;
      }
      
      case ScoringThree: {
        PIDPosition(elevatorMotor, ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case BallRemoverL2Primed: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_MIDDLE_ENDEFFECTOR);
        break;
      }

      case BallRemoverL2Go: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.DEALGAE_L2_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case BallRemoverL3Primed: {
        PIDPosition(elevatorMotor, ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.COMPLETLY_MIDDLE_ENDEFFECTOR);
        break;
      }

      case BallRemoverL3Go: {
        PIDPosition(elevatorMotor, ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endeffectorMotor, ElevatorConstants.DEALGAE_L2_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      default: {
        assert false;
      }
    }

  }

  public boolean elevatorAtReference() {
    // double elevatorRefrence = elevatorMotor.getClosedLoopReference().getValueAsDouble();
    double elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    double diffrence = elevatorRefrence - elevatorPosition;

    boolean headedUp = diffrence < 0;
    boolean forwardLimit = elevatorMotor.getForwardLimit().asSupplier().get().value == 0;
    boolean reverseLimit = elevatorMotor.getReverseLimit().asSupplier().get().value == 0;

    return (Math.abs(diffrence) <= 0.5 || (reverseLimit && headedUp) || (forwardLimit && !headedUp));
  }

  public boolean endeffectorAtReference() {
    // double elevatorRefrence = endefectorMotor.getClosedLoopReference().getValueAsDouble();
    double endeffectorPosition = endeffectorMotor.getPosition().getValueAsDouble();
    double diffrence = endeffectorRefrence - endeffectorPosition;

    boolean headedUp = diffrence < 0;
    boolean forwardLimit = endeffectorMotor.getForwardLimit().asSupplier().get().value == 0;
    boolean reverseLimit = endeffectorMotor.getReverseLimit().asSupplier().get().value == 0;

    return (Math.abs(diffrence) <= 0.5 || (reverseLimit && headedUp) || (forwardLimit && !headedUp));
  }
  // public void driveElevatorStick(Translation2d stick) {
  //   if (stick.getNorm() > 0.05) {
  //     elevatorMotor.set(stick.getY());
  //   }
  // }

  private void periodicWaiting() {
    if (!basinBeamBreak.get()) 
      transitionState(CoordinationState.Ready);
  }

  // private void periodicWaitingTripped() {
  //   if (!basinBeamBreak.get() && System.currentTimeMillis() > wait) 
  //     transitionState(CoordinationState.Ready);
  // }
  
  private void periodicReady() {
    if (elevatorAtReference())
      transitionState(CoordinationState.Hovering);
  }

  private void periodicScoring() {
    if (!endeffectorLimitSwitch.get()) transitionState(CoordinationState.Waiting);
  }

  public void manualElevatorVel(double velocity) {
    if (Math.abs(velocity) > 0.1) {
      elevatorMotor.set(velocity);
    }
    if (!elevatorManualStop) {
      elevatorManualStop = true;
      elevatorMotor.set(0);
    }
  }

  public void manualEndeffectorVel(double velocity) {
    if (Math.abs(velocity) > 0.1) {
      endeffectorMotor.set(velocity);
    }
    if (!endefectorManualStop) {
      endefectorManualStop = true;
      endeffectorMotor.set(0);
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Basin", basinBeamBreak.get() ? 1 : 0);
    SmartDashboard.putNumber("endefector", endeffectorLimitSwitch.get() ? 1 : 0);
    SmartDashboard.putString("State", currentState.toString());

    if (currentState == CoordinationState.Waiting) {
      periodicWaiting();
    } else if (currentState == CoordinationState.WatingBeamTripped) {
      // periodicWaitingTripped();
    } else if (currentState == CoordinationState.Ready) {
      periodicReady();
    }
    // } else if (currentState == CoordinationState.ScoringThree || currentState == CoordinationState.ScoringFour) {
    //   periodicScoring();
    // }
  }

  public boolean isL4Primed(){
    return currentState == CoordinationState.PrimedFour;
  }

  public boolean isL3Primed(){
    return currentState == CoordinationState.PrimedThree;
  }

  public void armShuffle(){
    if(!basinBeamBreak.get()){
      //shuffle the coral with the arm until coral hits beam break
    }
  }
}
