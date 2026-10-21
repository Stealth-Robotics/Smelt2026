package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class ShootCommand extends CommandBase {
    private final RobotSystem robot;

    private boolean startShoot = false;

    private static final Telemetry telemetry = StealthOpMode.telemetry;
    private double lastDistance = 0;

    private static final double DEFAULT_SHOOT_DISTANCE = 40;


    public ShootCommand(RobotSystem robot) {
        this.robot = robot;
    }

    @Override
    public void execute() {

        SingleShooterSubsystem shooter = robot.getShooter();
        CameraSubsystem camera = robot.getCamera();
        LLResult result = camera.getLastResult();
        if (result != null && result.isValid()) {
            lastDistance = camera.calcGoalDistanceByTa(result);
        }

        telemetry.addData("Last Distance", lastDistance);
        telemetry.addData("Calculated RPM", shooter.getRpmFromRange(lastDistance));

        if (startShoot) {
            if (lastDistance == 0) {
                lastDistance = DEFAULT_SHOOT_DISTANCE;
            }

            shooter.shoot(lastDistance);
        } else if (shooter.isActiveShot()) {
            shooter.stop();
        }

        // turn on feeder and open stops when shooter is ready
        if (shooter.isShootReady()) {
            robot.getIntakeSubsystem().setAutoIntake(true);

        } else if (robot.getIntakeSubsystem().getAutoIntake()){
            robot.getIntakeSubsystem().setAutoIntake(false);
        }
    }

    public void setStartShoot(boolean shoot) {
        this.startShoot = shoot;
    }

    public Command StartShootCommand(){
        return new InstantCommand(()-> setStartShoot(true));
    }

    public Command StopShootCommand(){
        return new InstantCommand(()-> setStartShoot(false));
    }

}
