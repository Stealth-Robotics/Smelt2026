package org.firstinspires.ftc.teamcode.paths;


import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class TestRedPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(62, 134, 270);
    private final Pose goShoot = poseFactory.of(62, 114, 260);
    private final Pose afterFirstShotStart = poseFactory.of(62, 114, 260);
    private final Pose afterFirstShot = poseFactory.of(72, 110, 0);

    public Path goShoot() {
        return Paths.line(start, goShoot).linear(start, goShoot);
    }

    public Path afterFirstShot() {
        return Paths.line(afterFirstShotStart, afterFirstShot).linear(afterFirstShotStart, afterFirstShot);
    }

    public Pose getStart() {
        return start;
    }
}