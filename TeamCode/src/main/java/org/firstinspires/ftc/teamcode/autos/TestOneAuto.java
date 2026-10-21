package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.TestRedPath;


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
                robot.getDriveSubsystem().followPathCommand(path.goShoot()),
                shootSequence(),
                robot.getDriveSubsystem().followPathCommand(path.afterFirstShot())
        );
    }
}
