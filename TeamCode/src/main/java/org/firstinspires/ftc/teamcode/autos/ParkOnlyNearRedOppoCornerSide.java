package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Only Near Red Oppo Corner Side")
public class ParkOnlyNearRedOppoCornerSide extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75);

    private final Pose start = poseFactory.of(9.266, 73.8429, 90);
    private final Pose curvedPark = poseFactory.of(15.2895, 93.1193, 180);
    private final Pose curvedParkControl1 = poseFactory.of(32.991, 97.8455, 0);

    public Path curvedPark() {
        return Paths.curve(start, curvedParkControl1, curvedPark).linear(start, curvedPark);
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
                drive.followPathCmd(curvedPark())
        );
    }
}
