package frc4388.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;
import frc4388.robot.subsystems.lidar.LidarIO.LidarState;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorState {
        public double elevatorRefrence;
        public double elevatorPosition;
        public boolean elevatorForwardLimit;
        public boolean elevatorReverseLimit;

        public double endeffectorRefrence;
        public double endeffectorPosition;
        public boolean endeffectorForwardLimit;
        public boolean endeffectorReverseLimit;

        
        public boolean basinBeamBreak;
        public boolean endeffectorLimitSwitch;
        public boolean intakeIR;

    }

    public default void elevatorToPosition(double position) {}
    public default void endeffectorToPosition(double position) {}
    public default void elevatorToVelocity(double velocity) {}
    public default void endeffectorToVelocity(double velocity) {}

    public default void updateInputs(ElevatorState state) {}
}