package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            //PID Drive
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0165, 0, 0.00125, 0.5, 0.02125))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.00825, 0, 0.00055, 0.5, 0.0125))

            //PID Heading
            .headingPIDFCoefficients(new PIDFCoefficients(1.01, 0, 0.02125, 0.0225))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.112, 0, 0.125, 0.0125))

            //PID Translational
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0575, 0, 0.001, 0.02125))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.045, 0.035))

            //Peso del robot
            .mass(9)

            //Desaceleración
            .forwardZeroPowerAcceleration(-42.40621181025876)
            .lateralZeroPowerAcceleration(-78.35042085332094)

            .centripetalScaling(0.000075);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.95, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(87.3770876906988)
            .yVelocity(74.30280309962475)

            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorName("rightFront")

            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorName("rightBack")

            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorName("leftFront")

            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorName("leftBack");

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6.33)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();



    }
}
