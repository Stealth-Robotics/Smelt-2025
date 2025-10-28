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
    private final CameraSubsystem cameraSubsystem;

    public static double AUTO_AIM_TOLERANCE = 0.3;
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.follower = Constants.createFollower(hardwareMap);
        this.cameraSubsystem = new CameraSubsystem(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleOpDrive();
    }

    public void update(){
        cameraSubsystem.update();
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


    private void drive(double x, double y, double rot, boolean isRobotCentric, boolean isAutoAim) {
        if (isAutoAim) {
            cameraSubsystem.getLastResult(); // Force an update of the Limelight data.
            Pose llPose = cameraSubsystem.getAvgTargetPose(50); // Get averaged target position.
            if (llPose != null) {
                // Calculate the turn power needed to center the target.
                //TODO: double output = getScaledTxOutput(llPose.getX(), AUTO_AIM_TOLERANCE);
                //TODO: shooterSys.setTargetRpmFromDistance(LimelightSubsystem.calcGoalDistanceByTy(llPose.getY()));
                //The output is applied to the rotation power (note: may need to be inverted).
                //TODO: turn = -output;

            } else {
                // If the target is lost, reset the PID controller to prevent integral windup.
                //headingController.reset();
            }
        }

        follower.setTeleOpDrive(x, y, rot,  isRobotCentric);
    }
    //TODO: make field centric toggleable
    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, BooleanSupplier isRobotCentric, BooleanSupplier isAutoAim) {
        telemetry.addData("Driving", x.getAsDouble());
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), -rot.getAsDouble(), isRobotCentric.getAsBoolean(), isAutoAim.getAsBoolean()));
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", AngleUnit.RADIANS.toDegrees(getHeading()));
        follower.update();
    }

}
