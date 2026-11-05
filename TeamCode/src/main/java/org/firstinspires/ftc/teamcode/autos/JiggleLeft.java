package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class JiggleLeft{
    public JiggleLeft(PoseFactory poseFactory) {
        this.poseFactory = poseFactory;
        start = poseFactory.of(13.1, 47, 0);
        jiggle1 = poseFactory.of(14.5, 47.5, 0);
        jiggle2Start = poseFactory.of(14.5, 47.5, 0);
        jiggle2 = poseFactory.of(14, 46.5, 0);
        jiggle3Start = poseFactory.of(14, 46.5, 0);
    }
    private final PoseFactory poseFactory;
    private final Pose start;
    private final Pose jiggle1;
    private final Pose jiggle2Start;
    private final Pose jiggle2;
    private final Pose jiggle3Start;

    public Path jiggle1() {
        return Paths.line(start, jiggle1).linear(start, jiggle1);
    }

    public Path jiggle2() {
        return Paths.line(jiggle2Start, jiggle2).linear(jiggle2Start, jiggle2);
    }

    public Path jiggle3() {
        return Paths.line(jiggle3Start, start).linear(jiggle3Start, start);
    }
}
