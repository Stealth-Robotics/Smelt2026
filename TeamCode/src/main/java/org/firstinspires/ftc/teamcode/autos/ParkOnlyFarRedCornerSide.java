package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Only Far Red Corner Side")
public class ParkOnlyFarRedCornerSide extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(11.7585, 133.247, 90);
    private final Pose justParkBro = poseFactory.of(11.8, 120, 90.1794);

    public Path justParkBro() {
        return Paths.line(start, justParkBro).reverseTangent();
    }

    @Override
    public void initialize()
    {
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand()
    {
        return new SequentialCommandGroup(
                drive.followPathCmd(justParkBro())
        );
    }
}
