package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class DriveSubsystem extends StealthSubsystem {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private final IMU imu;
    private double headingOffset = 0.0;

    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backRight = hardwareMap.get(DcMotorEx.class, "rightRear");

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(imuParameters);
    }

    public void setHeading(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
    }

    public void resetHeading() {
        headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    public void drive(double x, double y, double rot) {
        double heading = getHeading();
        double dx = x * Math.cos(-heading) - y * Math.sin(-heading);
        double dy = x * Math.sin(-heading) + y * Math.cos(-heading);

        dx *= 1.1; // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(dy) + Math.abs(dx) + Math.abs(rot), 1);
        double frontLeftPower = (dy + dx + rot) / denominator;
        double backLeftPower = (dy - dx + rot) / denominator;
        double frontRightPower = (dy - dx - rot) / denominator;
        double backRightPower = (dy + dx - rot) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), -rot.getAsDouble()));
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", AngleUnit.RADIANS.toDegrees(getHeading()));
    }
}
