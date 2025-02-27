package frc4388.robot.commands;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Command that is called if something is true
public class IfCommand extends InstantCommand {
    BooleanSupplier isTrue;
    Command trueCommand;
    Command falseCommand;

    public IfCommand(BooleanSupplier isTrue, Command trueCommand, Command falseCommand){
        this.isTrue = isTrue;
        this.trueCommand = trueCommand;
        this.falseCommand = falseCommand;
    }

    public IfCommand(BooleanSupplier isTrue, Command trueCommand){
        this(isTrue, trueCommand, Commands.none());
    }

    @Override
    public void execute() {
        if(isTrue.getAsBoolean())
            trueCommand.schedule();
        else
            falseCommand.schedule();
    }
}
