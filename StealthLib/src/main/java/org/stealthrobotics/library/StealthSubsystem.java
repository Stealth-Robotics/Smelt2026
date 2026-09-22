package org.stealthrobotics.library;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class StealthSubsystem extends SubsystemBase {
    public Command runOnce(Runnable action) {
        return Commands.runOnce(action, this);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted. Requires this
     * subsystem.
     *
     * @param action the action to run
     * @return the command
     * @see RunCommand
     */
    public Command run(Runnable action) {
        return Commands.run(action, this);
    }

    /**
     * Constructs a command that runs an action once and another action when the command is
     * interrupted. Requires this subsystem.
     *
     * @param start the action to run on start
     * @param end the action to run on interrupt
     * @return the command
     * @see StartEndCommand
     */
    public Command startEnd(Runnable start, Runnable end) {
        return Commands.startEnd(start, end, this);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted, and then runs a
     * second action. Requires this subsystem.
     *
     * @param run the action to run every iteration
     * @param end the action to run on interrupt
     * @return the command
     */
    public Command runEnd(Runnable run, Runnable end) {
        return Commands.runEnd(run, end, this);
    }
}
