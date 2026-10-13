package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.teamcode.pedro.Constants;


public class RobotSubsystem extends SubsystemBase {
    private final Follower follower;
    private final CameraSubsystem cameraSubsystem;


    public RobotSubsystem(HardwareMap hardwareMap){
        this.cameraSubsystem = new CameraSubsystem(hardwareMap);
        this.follower = Constants.create(hardwareMap);
    }

    public void drive(){ //make this the loop for driving that is in teleop
    }


}
