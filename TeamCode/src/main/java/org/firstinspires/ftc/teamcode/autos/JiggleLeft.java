package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class JiggleLeft{
    public JiggleLeft(PoseFactory poseFactory) {
        this.poseFactory = poseFactory;
        start = poseFactory.of(12, 47, 180);
        jiggle1 = poseFactory.of(13, 47, 180);
        jiggle2 = poseFactory.of(13, 46, 180);
        jiggle3Start = poseFactory.of(13, 46, 180);
    }
    private final PoseFactory poseFactory;
    private final Pose start;
    private final Pose jiggle1;

    private final Pose jiggle2;
    private final Pose jiggle3Start;

    public Path jiggle1() {
        return Paths.line(start, jiggle1).linear(start, jiggle1);
    }

    public Path jiggle2() {
        return Paths.line(jiggle1, jiggle2).linear(jiggle1, jiggle2);
    }

    public Path jiggle3() {
        return Paths.line(jiggle3Start, start).linear(jiggle3Start, start);
    }
}
