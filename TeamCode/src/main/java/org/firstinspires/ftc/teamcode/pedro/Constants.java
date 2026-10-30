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

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("goBildaPP");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.2932329553318773);
        c.yPodOffset.set(-6.49216959795614);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.18614280116058035);
//                Controller secondaryTranslationalForward = Controller.proportional(0.06877479120158314);
//                Controller primaryTranslationalLateral = Controller.proportional(0.24390111828559322);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.09011494604861789);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.017968640190348804));
//                c.brake.set(Controller.proportionalFeedforward(0.015273344161796482));
//
//                c.headingFeedback.set(Controller.proportional(3.4713961089931877));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05325312903703271, 0.0033392145928464495));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(-0.05054132266130609, 0.06884677014422538));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.006239061770360558, 0.0010564801658569842));
//
//                c.maxAchievableForwardVelocity.set(60.59014535413563);
//                c.maxAchievableStrafeVelocity.set(47.7181892603203);
//                c.naturalForwardDeceleration.set(40.52049303930872);
//                c.naturalStrafeDeceleration.set(66.62516998821941);
//            }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20170078379662246);
                Controller secondaryTranslationalForward = Controller.proportional(0.07452305006864829);
                Controller primaryTranslationalLateral = Controller.proportional(0.25756399581823064);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09516301421811602);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018178493988841856));
                c.brake.set(Controller.proportionalFeedforward(0.015451719890515577));

                c.headingFeedback.set(Controller.proportional(3.1806339747769075));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04839569170075206, 0.0036224406238930563));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07039243351189949, 0.05929368205257853));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011155829789794844, 0.0013492365433390612));

                c.maxAchievableForwardVelocity.set(57.69276137105969);
                c.maxAchievableStrafeVelocity.set(47.43616008350066);
                c.naturalForwardDeceleration.set(33.532963390805975);
                c.naturalStrafeDeceleration.set(58.23037665376004);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}