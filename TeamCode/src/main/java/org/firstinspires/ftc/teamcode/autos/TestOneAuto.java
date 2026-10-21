package org.firstinspires.ftc.teamcode.autos;

import static org.stealthrobotics.library.Commands.run;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.TestRedPath;
import org.stealthrobotics.library.opmodes.StealthOpMode;
import com.pedropathing.paths.Path;

@Autonomous (
            name = "Red Far Auto"
            )
public class TestOneAuto extends BioBuzzAutos {

    TestRedPath path = new TestRedPath();

    public void initialize() {
        super.initialize();
        follower.setPose(path.getStart());
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                driveCmd.followPathCommand(path.goShoot()),
                shootSequence(),
                driveCmd.followPathCommand(path.afterFirstShot())

        );

    }

    public Command followPathCmd(Path path){
        return new SequentialCommandGroup(new InstantCommand(()-> follower.follow(path)),
                new WaitUntilCommand(()-> follower.atParametricEnd()));

    }

}
