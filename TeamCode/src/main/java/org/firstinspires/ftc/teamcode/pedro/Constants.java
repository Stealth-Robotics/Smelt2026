package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    private static Follower follower = null;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("goBildaPP");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.503057524913878);
        c.yPodOffset.set(-6.070313341035618);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16167875368001391);
                Controller secondaryTranslationalForward = Controller.proportional(0.05973597934890173);
                Controller primaryTranslationalLateral = Controller.proportional(0.20521309843875915);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0758207564781421);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016986073183356626));
                c.brake.set(Controller.proportionalFeedforward(0.014438162205853132));

                c.headingFeedback.set(Controller.proportional(3.0624833484942933));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03875427058087863, 0.005904647976066221));

                c.linearBrakeCoefficients.set(Matrix.diag(0.04187925216442248, 0.03788999908370457));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018179224035117527, 0.0018169554082602464));

                c.maxAchievableForwardVelocity.set(59.71772073537703);
                c.maxAchievableStrafeVelocity.set(49.91471859453749);
                c.naturalForwardDeceleration.set(39.09910586942374);
                c.naturalStrafeDeceleration.set(70.38975401126757);
            }
    );
    private static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower getFollower(HardwareMap h) {
        if (follower == null) {
            follower = create(h);
        }

        return follower;
    }
}