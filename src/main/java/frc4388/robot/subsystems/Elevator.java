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
  private TalonFX endefectorMotor;

  private long wait = 0;
  private long maxWait = 1000;

  private double elevatorRefrence = 0;
  private double endefectorRefrence = 0;

  private DigitalInput basinBeamBreak;
  private DigitalInput endefectorLimitSwitch;

  public enum CoordinationState {
    Waiting, // for coral into the though
    WatingBeamTriped, //once the beam break trips
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
    if (motor == elevatorMotor) elevatorRefrence = position;
    else endefectorRefrence = position;

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
        wait = System.currentTimeMillis() + maxWait;
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case WatingBeamTriped: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_BEAM_BREAK_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case Ready: {
        PIDPosition(elevatorMotor, ElevatorConstants.GROUND_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case Hovering: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR);
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR);
        break;
      }

      case L2Score: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.L2_SCORE_ENDEFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }
      
      case PrimedFour: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_TOP_ENDEFECTOR);
        break;
      }

      case ScoringFour: {
        PIDPosition(elevatorMotor, ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.SCORING_FOUR_ENDEFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case PrimedThree: {
        PIDPosition(elevatorMotor, ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.PRIMED_THREE_ENDEFECTOR);
        break;
      }
      
      case ScoringThree: {
        PIDPosition(elevatorMotor, ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_DOWN_ENDEFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case BallRemoverL2Primed: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_MIDDLE_ENDEFECTOR);
        break;
      }

      case BallRemoverL2Go: {
        PIDPosition(elevatorMotor, ElevatorConstants.WAITING_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.DEALGAE_L2_EENDEFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      case BallRemoverL3Primed: {
        PIDPosition(elevatorMotor, ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.COMPLETLY_MIDDLE_ENDEFECTOR);
        break;
      }

      case BallRemoverL3Go: {
        PIDPosition(elevatorMotor, ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        PIDPosition(endefectorMotor, ElevatorConstants.DEALGAE_L2_EENDEFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        break;
      }

      default: {
        assert false;
      }
    }

  }

  public boolean elevatorAtRefrence() {
    // double elevatorRefrence = elevatorMotor.getClosedLoopReference().getValueAsDouble();
    double elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
    double diffrence = elevatorRefrence - elevatorPosition;

    boolean headedUp = diffrence < 0;
    boolean forwardLimit = elevatorMotor.getForwardLimit().asSupplier().get().value == 0;
    boolean reverseLimit = elevatorMotor.getReverseLimit().asSupplier().get().value == 0;

    return (Math.abs(diffrence) <= 0.5 || (reverseLimit && headedUp) || (forwardLimit && !headedUp));
  }
  
  public boolean endefectorAtRefrence() {
    // double elevatorRefrence = endefectorMotor.getClosedLoopReference().getValueAsDouble();
    double endefectorPosition = endefectorMotor.getPosition().getValueAsDouble();
    double diffrence = endefectorRefrence - endefectorPosition;

    boolean headedUp = diffrence < 0;
    boolean forwardLimit = endefectorMotor.getForwardLimit().asSupplier().get().value == 0;
    boolean reverseLimit = endefectorMotor.getReverseLimit().asSupplier().get().value == 0;

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
    if (elevatorMotor.getForwardLimit().asSupplier().get().value == 0)
      transitionState(CoordinationState.Hovering);
  }

  private void periodicScoring() {
    if (!endefectorLimitSwitch.get()) transitionState(CoordinationState.Waiting);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Basin", basinBeamBreak.get() ? 1 : 0);
    SmartDashboard.putNumber("endefector", endefectorLimitSwitch.get() ? 1 : 0);
    SmartDashboard.putString("State", currentState.toString());

    if (currentState == CoordinationState.Waiting) {
      periodicWaiting();
    } else if (currentState == CoordinationState.WatingBeamTriped) {
      // periodicWaitingTripped();
    } else if (currentState == CoordinationState.Ready) {
      periodicReady();
    }
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
