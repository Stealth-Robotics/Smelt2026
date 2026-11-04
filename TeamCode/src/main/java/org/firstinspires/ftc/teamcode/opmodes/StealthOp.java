package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.tools.AutoToTeleOpStorage;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;


public class StealthOp extends StealthOpMode {
    private GamepadEx driver;
    private GamepadEx operator;
    private RobotSystem robot;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        robot = new RobotSystem(hardwareMap, driver, operator);
        robot.configureBindings();
    }

    @TeleOp(name = "TeleOp Blue Alliance", group = "A")
    public static class BlueOp extends StealthOp {}

    @TeleOp(name = "TeleOp Red Alliance", group = "A")
    public static class RedOp extends StealthOp {}

    @Override
    public void whileWaitingToStart() {
        robot.updateCameraPipeline();
        Pose pose = robot.getDrive().getPose();

        telemetry.addData("Alliance", Alliance.get());
        telemetry.addData("Camera Pipeline", robot.getCamera().getCurrentPipeline());
        telemetry.addData("Stored Pose Valid", AutoToTeleOpStorage.isPoseValid());
        telemetry.addData("Current Pose", "Tx: %.2f, Ty: %.2f, Z: %.2f", pose.x(), pose.y(),  Math.toDegrees(pose.heading()));
    }

}
