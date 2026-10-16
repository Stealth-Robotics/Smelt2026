package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

// TODO:this is for testing a single shooter motor on the prototype bot
public class SingleShooterSubsystem extends StealthSubsystem {

    private final DcMotorEx shooterMotor1;
    private static final double MOTOR_TICKS_REV = 28;
    private final Telemetry telemetry = StealthOpMode.telemetry;
    private final ElapsedTime shootTimer = new ElapsedTime();

    private static final double VELOCITY_TOLERANCE_HIGH = 100;
    public static final double MAX_RPM = 5400;
    public static final double MIN_RPM = -4000;
    private static final double MIN_SHOOT_TIME_MS = 500;
    private static final double MAX_SHOOT_TIME_MS = 3000;
    private double targetRpm = 0;
    private double currentRpm = 0;

    private double floatPower = 0;

    private static final PIDFController shooterController = new PIDFController(0.005, 0, 0, .00041);

    public SingleShooterSubsystem(HardwareMap map) {
        shooterMotor1 = map.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {

        if (targetRpm != 0) {
            double calculatedPower= shooterController.calculate(getVelocity());
            setPower(calculatedPower);
            telemetry.addData("Calc Power   ", calculatedPower);
        } else {
            setPower(floatPower);
        }

        //TODO: remove unneeded prints statements
        telemetry.addData("Target RPM      ", getTargetRpm());
        telemetry.addData("Current RPM     ", getCurrentRpm());
        telemetry.addData("Current Power   ", getPower());
        telemetry.addData("Is shooter Ready", isShootReady());
        telemetry.addData("PIDF","P: %.5f, F:%.5f", shooterController.getP(), shooterController.getF());
    }

    public boolean isShootReady() {
        if (targetRpm == 0) return false;
        double difference = Math.abs(currentRpm - targetRpm);
        double curMs = shootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }

        return (difference <= VELOCITY_TOLERANCE_HIGH||curMs > MAX_SHOOT_TIME_MS);
    }

    public double getCurrentRpm() {
        currentRpm = tpsToRpm(shooterMotor1.getVelocity());
        return currentRpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void setController(double kp, double ki, double kd, double kf) {
        shooterController.setPIDF(kp, ki, kd, kf);
    }

    public void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }

        targetRpm = rpm;
        shootTimer.reset();
        shooterController.reset();
        shooterController.setSetPoint(rpmToTps(targetRpm));
    }

    public void setFloatPower(double power) {
        floatPower = power;
    }

    private double getPower() {
        return shooterMotor1.getPower();
    }
    private void setPower(double power) {
        shooterMotor1.setPower(power);
    }

    private double getVelocity() {
        return shooterMotor1.getVelocity();
    }

    private double tpsToRpm(double tps) {
        return Math.abs(tps * 60 / MOTOR_TICKS_REV);
    }

    private double rpmToTps( double rpm) {
        return Math.abs(rpm * MOTOR_TICKS_REV / 60);
    }
}
