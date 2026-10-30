package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class SimpleAuto extends BioBuzzAutos{

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(56, 36, 180);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shootSequence(45),
                drive.followPathCommand(path1()));
    }
}
