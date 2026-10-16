package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.stealthrobotics.library.Alliance;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotate;
    private boolean robotCentric = false;
    private boolean hold = false;
    private static final double slowPower = 0.3;
    private static final double maxPower = 1.0;

    private boolean slowMo = false;
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotate){
        this.driveSubsystem = driveSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
    }

    @Override
    public void execute(){
        Follower follower = driveSubsystem.getFollower();
        double invert = Alliance.isBlue() && !robotCentric ? -1 : 1;
        double currentPower = isSlowMo() ? slowPower : maxPower;
        double forwardPow = forward.getAsDouble() * currentPower * invert;
        double strafePow = strafe.getAsDouble() * currentPower * invert;
        double rotatePow = rotate.getAsDouble() * currentPower;
        DrivePowers powers = robotCentric?
                new DrivePowers(forwardPow, strafePow, rotatePow):
                ManualDrive.fieldCentric(forwardPow, strafePow, rotatePow, follower.pose().heading());

        if (hold) {
            ManualDrive.driveOrHold(follower, powers);
        } else {
            follower.manual(powers);
        }
    }

    public void toggleRobotCentric(){
        robotCentric = !robotCentric;
    }

    public boolean isRobotCentric() {
        return robotCentric;
    }

    public boolean isSlowMo() {
        return slowMo;
    }
    
    public void toggleSlowMo() {
        slowMo = ! slowMo;
    }

    public void toggleHoldMode() {
        hold = ! hold;
    }

    public boolean isHoldMode() {
        return hold;
    }

    public void setSlowMo(boolean pressed) {
        slowMo = pressed;
    }

}
