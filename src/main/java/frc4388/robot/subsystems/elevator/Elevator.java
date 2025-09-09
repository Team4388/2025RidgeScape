// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.constants.Constants.ElevatorConstants;
import frc4388.robot.constants.Constants.LEDConstants;
import frc4388.robot.subsystems.LED;
import frc4388.robot.subsystems.elevator.ElevatorIO.ElevatorState;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status.ReportLevel;

public class Elevator extends SubsystemBase implements Queryable {
  ElevatorIO io;
  ElevatorStateAutoLogged state = new ElevatorStateAutoLogged();

  /** Creates a new Elevator. */
  private LED led;

  @SuppressWarnings("unused")
  public long wait = 0;
  public long maxWait = 1000;

  public boolean elevatorManualStop = true;
  public boolean endefectorManualStop = true;

  public boolean disableAutoIntake = false;

  public boolean seededZeroEndefector = false;
  public boolean seededZeroElevator = false;

  // private ElevatorState state = new ElevatorState();

  public enum CoordinationState {
    Waiting, // for coral into the though
    WatingBeamTripped, //once the beam break trips
    Ready, // Has coral in endefector
    Hovering, // Has coral in endefector
    L2Score,
    L2ScoreLeave,
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

  // public Elevator(TalonFX elevatorTalonFX, TalonFX endeffectorTalonFX, DigitalInput basinLimitSwitch, DigitalInput endeffectorLimitSwitch, LED led) {
  public Elevator(ElevatorIO io, LED led) {
    this.io = io;
    this.led = led;

    currentState = CoordinationState.Ready;

    FaultReporter.register(this);
  }


  public void transitionState(CoordinationState state) {
    // elevatorMotor.enable();
    

    currentState = state;
    switch (currentState) {
      case Waiting: {
        wait = System.currentTimeMillis() + maxWait;
        io.elevatorToPosition(ElevatorConstants.WAITING_POSITION_ELEVATOR);
        io.endeffectorToPosition(ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR + (!seededZeroEndefector ? 10 : 0));
        led.setMode(LEDConstants.WAITING_PATTERN);
        break;
      }

      case WatingBeamTripped: {
        io.elevatorToPosition(ElevatorConstants.WAITING_POSITION_BEAM_BREAK_ELEVATOR);
        io.endeffectorToPosition(ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        led.setMode(LEDConstants.DOWN_PATTERN);
        break;
      }

      case Ready: {
        io.elevatorToPosition(ElevatorConstants.GROUND_POSITION_ELEVATOR + (!seededZeroElevator ? 10 : 0));
        io.endeffectorToPosition(ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        led.setMode(LEDConstants.DOWN_PATTERN);
        break;
      }

      case Hovering: {
        io.elevatorToPosition(ElevatorConstants.HOVERING_POSITION_ELEVATOR);
        io.endeffectorToPosition(ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR);
        led.setMode(LEDConstants.READY_PATTERN);
        break;
      }

      case L2Score: {
        io.elevatorToPosition(ElevatorConstants.L2_SCORE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.L2_SCORE_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case L2ScoreLeave: {
        io.elevatorToPosition(ElevatorConstants.L2_LEAVE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.L2_SCORE_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }
      
      case PrimedFour: {
        io.elevatorToPosition(ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.PRIMED_FOUR_ENDEFFECTOR);
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case ScoringFour: {
        io.elevatorToPosition(ElevatorConstants.MAX_POSITION_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.SCORING_FOUR_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case PrimedThree: {
        io.elevatorToPosition(ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.PRIMED_THREE_ENDEFFECTOR);
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }
      
      case ScoringThree: {
        io.elevatorToPosition(ElevatorConstants.SCORING_THREE_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.COMPLETLY_DOWN_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case BallRemoverL2Primed: {
        io.elevatorToPosition(ElevatorConstants.DEALGAE_L2_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.DEALGAE_L2_ENDEFFECTOR);
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case BallRemoverL2Go: {
        io.elevatorToPosition(ElevatorConstants.DEALGAE_L2_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.DEALGAE_L2_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      case BallRemoverL3Primed: {
        io.elevatorToPosition(ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.DEALGAE_L2_ENDEFFECTOR);
        break;
      }

      case BallRemoverL3Go: {
        io.elevatorToPosition(ElevatorConstants.DEALGAE_L3_ELEVATOR + AutoConstants.ELEVATOR_OFFSET_TRIM.get());
        io.endeffectorToPosition(ElevatorConstants.DEALGAE_L2_ENDEFFECTOR + AutoConstants.ARM_OFFSET_TRIM.get());
        led.setMode(LEDConstants.SCORING_PATTERN);
        break;
      }

      default: {
        assert false;
      }
    }

  }

  public void togggleAutoIntaking() {
    disableAutoIntake = !disableAutoIntake;
  }

  public boolean elevatorAtReference() {
    double diffrence = state.elevatorRefrence - state.elevatorPosition;

    boolean headedUp = diffrence < 0;

    return (Math.abs(diffrence) <= 0.5 
      || (state.elevatorReverseLimit && headedUp) 
      || (state.elevatorForwardLimit && !headedUp)
    );
  }

  public boolean endeffectorAtReference() {
    double diffrence = state.endeffectorRefrence - state.endeffectorPosition;

    boolean headedUp = diffrence < 0;

    return (Math.abs(diffrence) <= 0.5 
      || (state.elevatorReverseLimit && headedUp) 
      || (state.endeffectorForwardLimit && !headedUp)
    );
  }
  // public void driveElevatorStick(Translation2d stick) {
  //   if (stick.getNorm() > 0.05) {
  //     elevatorMotor.set(stick.getY());
  //   }
  // }

  public boolean getEndeffectorLimit() {
    return state.endeffectorLimitSwitch;
  }

  private void periodicWaiting() {
    if (!state.basinBeamBreak) 
      transitionState(CoordinationState.Ready);
    // if(!endeffectorLimitSwitch.get())
    //   transitionState(CoordinationState.Hovering);
  }

  // private void periodicWaitingTripped() {
  //   if (!basinBeamBreak.get() && System.currentTimeMillis() > wait) 
  //     transitionState(CoordinationState.Ready);
  // }
  
  private void periodicReady() {
    if (elevatorAtReference() && !state.endeffectorLimitSwitch)
      transitionState(CoordinationState.Hovering);
    if(elevatorAtReference() && state.endeffectorLimitSwitch)
      transitionState(CoordinationState.Hovering);
  }

  @SuppressWarnings("unused")
  private void periodicScoring() {
    if (!state.endeffectorLimitSwitch) 
      transitionState(CoordinationState.Waiting);
  }

  public void manualElevatorVel(double velocity) {
    if (Math.abs(velocity) > 0.1) {
      io.elevatorToVelocity(velocity);
      elevatorManualStop = false;
      return;
    }
    if (!elevatorManualStop) {
      elevatorManualStop = true;
      io.elevatorToVelocity(0);
    }
  }

  public void manualEndeffectorVel(double velocity) {
    if (Math.abs(velocity) > 0.1) {
      io.endeffectorToVelocity(velocity);
      endefectorManualStop = false;
      return;
    }
    if (!endefectorManualStop) {
      endefectorManualStop = true;
      io.endeffectorToVelocity(0);
    }
  }

  @Override
  public void periodic() {

    // double elevatorVelocity = elevatorMotor.getVelocity().getValueAsDouble();
    // double elevatorTorque = elevatorMotor.getTorqueCurrent().getValueAsDouble();
    // double endeffectorVelocity = endeffectorMotor.getVelocity().getValueAsDouble();
    // double endeffectorTorque = endeffectorMotor.getTorqueCurrent().getValueAsDouble();


    // if(endeffectorVelocity < ElevatorConstants.SAFETY_ENDEFFECTOR_MIN_VELOCITY && endeffectorTorque > ElevatorConstants.SAFETY_ENDEFFECTOR_MAX_TORQUE ){
    //   PIDPosition(endeffectorMotor, endeffectorMotor.getPosition().getValueAsDouble());
    // }

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Velocity Endeffector", endeffectorVelocity);
    // SmartDashboard.putNumber("Torque Endeffector", endeffectorTorque);
    // SmartDashboard.putNumber("Basin", basinBeamBreak.get() ? 1 : 0);
    // SmartDashboard.putNumber("endefector", endeffectorLimitSwitch.get() ? 1 : 0);
    // SmartDashboard.putNumber("intake", intakeIR.get() ? 1 : 0);
    // SmartDashboard.putString("State", currentState.toString());

    io.updateInputs(state);
    Logger.processInputs("Elevator", state);
    
    if (!seededZeroEndefector && state.endeffectorForwardLimit) {
      io.endeffectorToPosition(0);
      seededZeroEndefector = !seededZeroEndefector;
    }

    if (!seededZeroElevator && state.elevatorReverseLimit) {
      io.endeffectorToPosition(0);
      seededZeroElevator = !seededZeroElevator;
    }
    
    if (disableAutoIntake) return;

    if (currentState == CoordinationState.Waiting) {
      periodicWaiting();
    } else if (currentState == CoordinationState.WatingBeamTripped) {
      // periodicWaitingTripped();
    } else if (currentState == CoordinationState.Ready) {
      periodicReady();
    }

    if(!state.intakeIR){
      led.setMode(LEDConstants.DOWN_PATTERN);
    }

    
    // } else if (currentState == CoordinationState.ScoringThree || currentState == CoordinationState.ScoringFour) {
    //   periodicScoring();
    // }
  }

  @AutoLogOutput(key="Elevator/state")
  public String getState() {
    return currentState.toString();
  }

  public boolean isL4Primed() {
    return currentState == CoordinationState.PrimedFour;
  }

  public boolean isL3Primed() {
    return currentState == CoordinationState.PrimedThree;
  }

  public boolean hasCoral() {
    return elevatorAtReference() && currentState == CoordinationState.Hovering || !state.endeffectorLimitSwitch;
  }

  public void elevatorStop() {
    io.elevatorToVelocity(0);
  }

  public void endeffectorStop() {
    io.endeffectorToVelocity(0);
  }

  public boolean readyToMove() {
    return !state.intakeIR || hasCoral() || !state.endeffectorLimitSwitch;
    // return hasCoral();
  }

  public void armShuffle(){
    if(!state.basinBeamBreak){
      //shuffle the coral with the arm until coral hits beam break
    }
  }

  @Override
  public String getName() {
    return "Elevator";
  }

  // @Override
  // public void queryStatus() {}

  @Override
  public Status diagnosticStatus() {
    Status status = new Status();

    status.addReport(ReportLevel.INFO, "Elevator Mode: " + currentState.name());

    return status;
  }
}
