package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="Simple RED")
public class SimpleAuto extends BioBuzzAutos{

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose path1 = poseFactory.of(56.0803, 11.8978, 270);
    private final Pose point2 = poseFactory.of(23.866, 46.7068, 178.5817);
    private final Pose point2Control1 = poseFactory.of(37.4384, 46.3708, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2).tangent();
    }

    public void initialize() {
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                //shootSequence(45),
                drive.followPathCmd(path1()),
                drive.followPathCmd(path2()));
    }
}
