package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class DriveSubsystem extends StealthSubsystem {

    private final Follower follower;
    private double headingOffset = 0.0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleOpDrive();
    }

    public void setHeading(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public double getHeading() {
        return follower.getHeading();
    }

    public Follower getFollower(){
        return follower;
    }
    public void resetHeading(){
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        }catch(InterruptedException e){
            //empty block
        }
    }


    private void drive(double x, double y, double rot, boolean isRobotCentric) {
        follower.setTeleOpDrive(x, y, rot,  isRobotCentric);
    }
    //TODO: make field centric toggleable
    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier isRobotCentric) {
        telemetry.addData("Driving", x.getAsDouble());
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), -rot.getAsDouble(), isRobotCentric.getAsBoolean()));
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", AngleUnit.RADIANS.toDegrees(getHeading()));
        follower.update();
    }

}
