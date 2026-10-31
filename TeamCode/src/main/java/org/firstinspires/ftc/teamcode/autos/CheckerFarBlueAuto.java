package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.stealthrobotics.library.commands.IfCommand;

@Autonomous(name = "Checker Far Blue Auto")
public class CheckerFarBlueAuto extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees()
            .rotateAround(FIELD_CENTER,180);

    private final Pose start = poseFactory.of(56.8844, FieldConstants.ROBOT_CENTER_Y, 270);
    private final Pose moveUp = poseFactory.of(56.838, 12.3968, 270);
    private final Pose alignFlower = poseFactory.of(29.6241, 47, 86.73);
    private final Pose alignFlowerControl1 = poseFactory.of(28.318, 24.1395, 0);
    private final Pose harvest = poseFactory.of(12, 47, 180);
    private final Pose postHarvest = poseFactory.of(18, 47, 180);
    private final Pose goShootToo = poseFactory.of(53.31, 10.3272, -46.0846);
    private final Pose checkTagAim = poseFactory.of(53.31, 15.0612, -90);
    private final Pose aimOtherSideStart = poseFactory.of(53.31, 15.0612, 90);
    private final Pose aimOtherSide = poseFactory.of(40.3475, 126.177, 120);
    private final Pose aimOtherSideControl1 = poseFactory.of(75.9662, 90.7934, 0);
    private final Pose parkIt = poseFactory.of(11.8202, 119.4826, -166.7936);

    public Path moveUp() {
        return Paths.line(start, moveUp).linear(start, moveUp);
    }

    public Path alignFlower() {
        return Paths.curve(moveUp, alignFlowerControl1, alignFlower).tangent();
    }

    public Path harvest() {
        return Paths.line(alignFlower, harvest).tangent();
    }

    public Path postHarvest() {
        return Paths.line(harvest, postHarvest).reverseTangent();
    }

    public Path goShootToo() {
        return Paths.line(postHarvest, goShootToo).tangent();
    }

    public Path checkTagAim() {
        return Paths.line(goShootToo, checkTagAim).reverseTangent();
    }

    public Path aimOtherSide() {
        return Paths.curve(aimOtherSideStart, aimOtherSideControl1, aimOtherSide).linear(aimOtherSideStart, aimOtherSide);
    }

    public Path parkIt() {
        return Paths.line(aimOtherSide, parkIt).tangent();
    }


    @Override
    public void initialize(){
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shootSequenceCmd(58.5),
                drive.followPathCmd(moveUp()),
                drive.followPathCmd(alignFlower()),
                robot.getHarvesterSubsystem().harvDownCmd(),
                intake.startAutoIntakeCommand(true),
                drive.followPathCmd(harvest()),
                waitCmd(2000),
                drive.followPathCmd(postHarvest()),
                intake.setAutoIntakeCommand(false),
                robot.getHarvesterSubsystem().harvUpCmd(),
                intake.stopAutoIntakeCommand(),
                drive.followPathCmd(goShootToo()),
                drive.followPathCmd(checkTagAim()),
                waitForTagsCmd(1000),

                new IfCommand(this::getTagSeen,
                        // Tags Seen == TRUE
                        new SequentialCommandGroup(
                                shooter.startShootCmd(57),
                                drive.followPathCmd(aimOtherSide())

                        ),
                        // FALSE NO TAGS SEEN GO AND SHOOT OTHER SIDE
                        new SequentialCommandGroup(
                                drive.followPathCmd(aimOtherSide()),
                                shootSequenceCmd())
                ),

                drive.followPathCmd(parkIt())
        );
    }
}
