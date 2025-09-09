package frc4388.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.constants.Constants.ElevatorConstants;

public class ElevatorReal implements ElevatorIO {
    TalonFX elevatorMotor;
    TalonFX endeffectorMotor;
    
    DigitalInput basinLimitSwitch;
    DigitalInput endeffectorLimitSwitch;
    DigitalInput intakeIR;

    
    double elevatorRefrence = 0;
    double endeffectorRefrence = 0;

    public ElevatorReal(TalonFX elevatorTalonFX, TalonFX endeffectorTalonFX, DigitalInput basinLimitSwitch, DigitalInput endeffectorLimitSwitch, DigitalInput intakeIR) {
        this.elevatorMotor = elevatorTalonFX;
        this.endeffectorMotor = endeffectorTalonFX;

        this.basinLimitSwitch = basinLimitSwitch;
        this.endeffectorLimitSwitch = endeffectorLimitSwitch;
        this.intakeIR = intakeIR;

        
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        endeffectorMotor.setNeutralMode(NeutralModeValue.Brake);
        
        elevatorMotor.getConfigurator().apply(ElevatorConstants.ELEVATOR_PID);
        endeffectorMotor.getConfigurator().apply(ElevatorConstants.ENDEFFECTOR_PID);
    }

    @Override
    public void updateInputs(ElevatorState state) {
        state.elevatorPosition = elevatorMotor.getPosition().getValueAsDouble();
        state.elevatorRefrence = elevatorRefrence;
        state.elevatorForwardLimit = elevatorMotor.getForwardLimit().asSupplier().get().value == 0;
        state.elevatorReverseLimit = elevatorMotor.getReverseLimit().asSupplier().get().value == 0;

        state.endeffectorPosition = endeffectorMotor.getPosition().getValueAsDouble();
        state.endeffectorRefrence = endeffectorRefrence;
        state.endeffectorForwardLimit = endeffectorMotor.getForwardLimit().asSupplier().get().value == 0;
        state.endeffectorReverseLimit = endeffectorMotor.getReverseLimit().asSupplier().get().value == 0;


        state.basinBeamBreak = basinLimitSwitch.get();
        state.endeffectorLimitSwitch = endeffectorLimitSwitch.get();
        state.intakeIR = intakeIR.get();
    }

    @Override
    public void elevatorToPosition(double position) {
        elevatorRefrence = position; 
        var request = new PositionDutyCycle(position);
        elevatorMotor.setControl(request);
    }

    @Override
    public void endeffectorToPosition(double position) {
        endeffectorRefrence = position;
        var request = new PositionDutyCycle(position);
        endeffectorMotor.setControl(request);
    }

    @Override
    public void elevatorToVelocity(double velocity) {
        elevatorMotor.set(velocity);
    }

    
    @Override
    public void endeffectorToVelocity(double velocity) {
        endeffectorMotor.set(velocity);
    }
}
