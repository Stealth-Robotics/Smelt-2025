package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Drivetrain;
import com.pedropathing.VectorCalculator;
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
            .mass(11.10)
            .forwardZeroPowerAcceleration(-40.33)
            .lateralZeroPowerAcceleration(-70.75)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0 , 0.004, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5, 0, 0.003, 0.6, 0.03))
            .centripetalScaling(0.0005);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.2,
            1.4
    );
    public static PinpointConstants localizer = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(5)
            .yawScalar(1)
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.85)
            .yVelocity(39);
    public static Follower createFollower(HardwareMap hardwareMap) {
        Follower f = new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizer)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .build();
        f.activateDrive();
        f.activateHeading();
        return f;
    }
}
