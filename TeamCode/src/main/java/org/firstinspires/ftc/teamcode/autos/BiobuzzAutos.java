package org.firstinspires.ftc.teamcode.autos;

import android.view.contentcapture.DataRemovalRequest;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class BiobuzzAutos extends StealthOpMode {

    RobotSystem robot;
    Follower follower;
    DriveSubsystem drive;
    @Override
    public void initialize() {
        robot = new RobotSystem(hardwareMap);
        drive = robot.getDrive();
        follower = drive.getFollower();
    }
}
