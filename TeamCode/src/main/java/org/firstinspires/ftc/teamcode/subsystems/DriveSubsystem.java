package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.follower.Follower;
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

    private final Follower follower;
    private double headingOffset = 0.0;

    public DriveSubsystem(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
    }

    public void setHeading(double headingOffset) {
        this.headingOffset = headingOffset;
    }

    public double getHeading() {
        return follower.getHeading();
    }

    public void resetHeading(){
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        }catch(InterruptedException e){
            //empty block
        }
    }


    public void drive(double x, double y, double rot) {
        follower.setTeleOpDrive(x, y, rot,  false);
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), -rot.getAsDouble()));
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading", AngleUnit.RADIANS.toDegrees(getHeading()));
    }
}
