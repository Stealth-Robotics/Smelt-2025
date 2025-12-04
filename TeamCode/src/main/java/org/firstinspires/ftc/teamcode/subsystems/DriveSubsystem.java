package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.usablethings.Drawing;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.bylazar.configurables.annotations.Configurable;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Configurable
public class DriveSubsystem extends StealthSubsystem {

    private final Follower follower;
    private final TelemetryManager telemetryM;
    private double headingOffset = 0.0;
    private final CameraSubsystem cameraSubsystem;
    public static final double MAX_ROTATION_POWER = 0.90;

    public static final double MIN_ROTATION_POWER = 0.068 ;
    private double offset = 0;
    private static final PIDFCoefficients HEADING_COEFFICIENTS
            = new PIDFCoefficients(0.018, 0.0001, 0.001, .02); //0.018, 0.0001, 0.001, 0.02
    private static final PIDFController headingController = new PIDFController(HEADING_COEFFICIENTS);
    public static double AUTO_AIM_TOLERANCE = 0.2;
    public DriveSubsystem(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
        this.cameraSubsystem = new CameraSubsystem(hardwareMap);
        follower.startTeleOpDrive(true);
        Drawing.init();
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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
    public void setAlliance(boolean isAllianceBlue){
        if(isAllianceBlue){
            offset = -1.5 ;
        }
        else{
            offset = 1.5;
        }
    }
    public void setPose(Pose pose){
        follower.setPose(pose);
    }
    public boolean doAimAtTarget(double tolerance, double offset, long latency) {
        if (!follower.isTeleopDrive()) {
            follower.startTeleopDrive(true);
        }

        cameraSubsystem.getLastResult(); // Force an update of the Limelight data.
        Pose llPose = cameraSubsystem.getAvgTargetPose(latency); // Get averaged target position.
        double turn = 0;
        if (llPose != null) {
            // Calculate the turn power needed to center the target.
            double output = getScaledTxOutput(llPose.getX() + offset, tolerance);
            turn = output;
        } else {
            // If the target is lost, reset the PID controller to prevent integral windup.
            headingController.reset();
        }

        follower.setTeleOpDrive(0, 0, turn, false);
        return turn == 0 && llPose != null;
    }
    private void drive(double y, double x, double rot, boolean isAutoAim, double offset) {
        double turn = rot;
        if (isAutoAim) {
            cameraSubsystem.getLastResult(); // Force an update of the Limelight data.
            Pose llPose = cameraSubsystem.getAvgTargetPose(50); // Get averaged target position.
            if (llPose != null) {
                // Calculate the turn power needed to center the target.
                double output = getScaledTxOutput(llPose.getX() + offset , AUTO_AIM_TOLERANCE);
                //The output is applied to the rotation power (note: may need to be inverted).
                turn = output;

            } else {
                // If the target is lost, reset the PID controller to prevent integral windup.
                headingController.reset();
            }
        }

        follower.setTeleOpDrive(y, x, turn,  this.robotCentric);
        telemetryM.addData("TURN:", turn);
        follower.update();
    }
    public Command  FollowPath(PathChain path, boolean holdPoint){
        return this.runOnce(()-> follower.followPath(path,holdPoint))
                        .andThen(new WaitUntilCommand(()-> !follower.isBusy()));
    }
    //TODO: make field centric toggleable
    //x is reversed
    private boolean robotCentric = false;
    public Command driveTeleop(DoubleSupplier y, DoubleSupplier x, DoubleSupplier rot, BooleanSupplier isAutoAim) {
        telemetry.addData("Driving", y.getAsDouble());

        return this.run(() -> drive(y.getAsDouble(), -x.getAsDouble(), rot.getAsDouble(), isAutoAim.getAsBoolean(), offset));
    }

    public Command swapDriveMode()
    {
       return this.runOnce(() -> robotCentric = !robotCentric);
    }
    private double getScaledTxOutput(double txDelta, double tolerance) {

        if (Math.abs(txDelta) < tolerance) {
            return 0;
        }

        // Set the PID controller's target to 0 (no error).
        headingController.setTargetPosition(0);
        // Update the controller with the current error.
        headingController.updateError(-txDelta);
        // Run the PID calculation.
        double pidOutput = headingController.run();

        // If the error is outside the tolerance, return the clamped PID output.
        // Otherwise, return 0 to stop turning.
        if (Math.abs(pidOutput) < MIN_ROTATION_POWER) {
            if (pidOutput < 0) {
                return -MIN_ROTATION_POWER;
            } else {
                return MIN_ROTATION_POWER;
            }
        }
        return  MathFunctions.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }
    private void draw() {
        Drawing.drawDebug(follower);
    }

    @Override
    public void periodic() {
        telemetryM.addData("Raw Heading", getHeading());
        telemetryM.addData("Heading", Math.toDegrees(getHeading()));
        telemetryM.addData("isRobotCentric", robotCentric);
        telemetryM.addData("X Pose", follower.getPose().getX());
        telemetryM.addData("Y Pose", follower.getPose().getY());
        telemetryM.addData("offset", offset);
        draw();
        follower.update();
    }

}
