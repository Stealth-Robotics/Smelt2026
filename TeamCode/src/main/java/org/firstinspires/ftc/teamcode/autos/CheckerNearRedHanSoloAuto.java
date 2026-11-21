//Reminder: No pose factory rotation in red

package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.tools.FieldConstants;
import org.firstinspires.ftc.teamcode.tools.Targets;
import org.stealthrobotics.library.commands.IfCommand;

@Autonomous(name = "Checker Near Red Han Solo Auto", group = "RED")
public class CheckerNearRedHanSoloAuto extends BioBuzzAutos
{
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.9, FieldConstants.ROBOT_CENTER_Y, 270);
    private final Pose moveUp = poseFactory.of(56.31, 12.06, 270);
    private final Pose alignFlower = poseFactory.of(29.6241, 47, 155.9755);
    private final Pose alignFlowerControl1 = poseFactory.of(42.2343, 41.3791, 0);
    private final Pose harvest = poseFactory.of(12, 47, 180);
    private final Pose postHarvest = poseFactory.of(18, 47, 180);
    private final Pose goShootToo = poseFactory.of(53.31, 10.3272, 133.9154);
    private final Pose checkTagAim = poseFactory.of(53.31, 15.0612, -90);
    private final Pose aimOtherSideStart = poseFactory.of(53.31, 15.0612, 90);
    private final Pose aimOtherSide = poseFactory.of(40.3475, 122.18, 125);
    private final Pose aimOtherSideControl1 = poseFactory.of(75.9662, 90.7934, 0);
    private final Pose harvestTooStart = poseFactory.of(40.3475, 122.18, 125);
    private final Pose harvestToo = poseFactory.of(46.7504, 129.5, 90);
    private final Pose postHarvestTooStart = poseFactory.of(46.7504, 130.1992, 90);
    private final Pose postHarvestToo = poseFactory.of(40.8922, 122.18, 125);
    private final Pose parkIt = poseFactory.of(11.6124, 121.9751, -179.5991);

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
        return Paths.line(postHarvest, goShootToo).reverseTangent();
    }

    public Path checkTagAim() {
        return Paths.line(goShootToo, checkTagAim).reverseTangent();
    }

    public Path aimOtherSide() {
        return Paths.curve(aimOtherSideStart, aimOtherSideControl1, aimOtherSide).linear(aimOtherSideStart, aimOtherSide);
    }

    public Path harvestToo() {
        return Paths.line(harvestTooStart, harvestToo).linear(harvestTooStart, harvestToo);
    }

    public Path postHarvestToo() {
        return Paths.line(postHarvestTooStart, postHarvestToo).linear(postHarvestTooStart, postHarvestToo);
    }

    public Path parkIt() {
        return Paths.line(postHarvestToo, parkIt).tangent();
    }

    @Override
    public void initialize()
    {
        super.initialize();
        follower.setPose(start);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                shootSequenceCmd(), // 62
                drive.followPathCmd(moveUp()),
                drive.followPathCmd(alignFlower()),
                startHarvestingCmd(),
                drive.followPathCmd(harvest(),3000),
                waitCmd(2000),
                drive.followPathCmd(postHarvest()),
                stopHarvestingCmd(),
                intake.stopAutoIntakeCommand(),
                drive.followPathCmd(goShootToo()),
                drive.followPathCmd(checkTagAim()),
                waitForTagsCmd(500),

                new IfCommand(this::getTagSeen,
                        // Tags Seen == TRUE
                        new SequentialCommandGroup
                                (
                                        shootSequenceCmd(), //57
                                        drive.followPathCmd(aimOtherSide()),
                                        drive.followPathCmd(harvestToo()),
                                        drive.followPathCmd(postHarvestToo()),
                                        shootSequenceCmd()
                                ),
                        // FALSE NO TAGS SEEN GO AND SHOOT OTHER SIDE
                        new SequentialCommandGroup
                                (
                                        drive.followPathCmd(aimOtherSide()),
                                        drive.followPathCmd(harvestToo()),
                                        drive.followPathCmd(postHarvestToo()),
                                        shootSequenceCmd() //57
                                )
                ),

                drive.followPathCmd(parkIt())
        );
    }
}
