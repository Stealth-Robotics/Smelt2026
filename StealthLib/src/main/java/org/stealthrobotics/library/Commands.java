package org.stealthrobotics.library;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Namespace for command factory methods.
 *
 * <p>For convenience, you might want to static import the members of this class.
 */
public final class Commands {
    /**
     * Constructs a command that does nothing, finishing immediately.
     *
     * @return the command
     */
    public static Command none() {
        return new InstantCommand();
    }

    /**
     * Constructs a command that does nothing until interrupted.
     *
     * @param requirements Subsystems to require
     * @return the command
     */
    public static Command idle(Subsystem... requirements) {
        return run(() -> {}, requirements);
    }

    // Action org.stealthrobotics.library.Commands

    /**
     * Constructs a command that runs an action once and finishes.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see InstantCommand
     */
    public static Command runOnce(Runnable action, Subsystem... requirements) {
        return new InstantCommand(action, requirements);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted.
     *
     * @param action the action to run
     * @param requirements subsystems the action requires
     * @return the command
     * @see RunCommand
     */
    public static Command run(Runnable action, Subsystem... requirements) {
        return new RunCommand(action, requirements);
    }

    /**
     * Constructs a command that runs an action once and another action when the command is
     * interrupted.
     *
     * @param start the action to run on start
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     * @see StartEndCommand
     */
    public static Command startEnd(Runnable start, Runnable end, Subsystem... requirements) {
        return new StartEndCommand(start, end, requirements);
    }

    /**
     * Constructs a command that runs an action every iteration until interrupted, and then runs a
     * second action.
     *
     * @param run the action to run every iteration
     * @param end the action to run on interrupt
     * @param requirements subsystems the action requires
     * @return the command
     */
    public static Command runEnd(Runnable run, Runnable end, Subsystem... requirements) {

        return new FunctionalCommand(
                () -> {}, run, interrupted -> end.run(), () -> false, requirements);
    }

    /**
     * Constructs a command that prints a message and finishes.
     *
     * @param message the message to print
     * @return the command
     * @see PrintCommand
     */
    public static Command print(String message) {
        return new PrintCommand(message);
    }

    // Idling org.stealthrobotics.library.Commands

    /**
     * Constructs a command that does nothing, finishing after a specified duration.
     *
     * @param seconds after how long the command finishes
     * @return the command
     * @see WaitCommand
     */
    public static Command waitSeconds(long seconds) {
        return new WaitCommand(seconds);
    }

    /**
     * Constructs a command that does nothing, finishing once a condition becomes true.
     *
     * @param condition the condition
     * @return the command
     * @see WaitUntilCommand
     */
    public static Command waitUntil(BooleanSupplier condition) {
        return new WaitUntilCommand(condition);
    }

    // Selector org.stealthrobotics.library.Commands

    /**
     * Runs one of two commands, based on the boolean selector function.
     *
     * @param onTrue the command to run if the selector function returns true
     * @param onFalse the command to run if the selector function returns false
     * @param selector the selector function
     * @return the command
     * @see ConditionalCommand
     */
    public static Command either(Command onTrue, Command onFalse, BooleanSupplier selector) {
        return new ConditionalCommand(onTrue, onFalse, selector);
    }






    // Command Groups

    /**
     * Runs a group of commands in series, one after the other.
     *
     * @param commands the commands to include
     * @return the command group
     * @see SequentialCommandGroup
     */
    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }



    /**
     * Runs a group of commands at the same time. Ends once all commands in the group finish.
     *
     * @param commands the commands to include
     * @return the command
     * @see ParallelCommandGroup
     */
    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    /**
     * Runs a group of commands at the same time. Ends once any command in the group finishes, and
     * cancels the others.
     *
     * @param commands the commands to include
     * @return the command group
     * @see ParallelRaceGroup
     */
    public static Command race(Command... commands) {
        return new ParallelRaceGroup(commands);
    }

    /**
     * Runs a group of commands at the same time. Ends once a specific command finishes, and cancels
     * the others.
     *
     * @param deadline the deadline command
     * @param otherCommands the other commands to include
     * @return the command group
     * @see ParallelDeadlineGroup
     * @throws IllegalArgumentException if the deadline command is also in the otherCommands argument
     */
    public static Command deadline(Command deadline, Command... otherCommands) {
        return new ParallelDeadlineGroup(deadline, otherCommands);
    }

    private Commands() {
        throw new UnsupportedOperationException("This is a utility class");
    }
}