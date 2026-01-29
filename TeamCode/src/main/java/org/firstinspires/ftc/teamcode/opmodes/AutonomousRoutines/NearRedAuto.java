package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "NearRedAuto", group = "Autos", preselectTeleOp = "Teleop")
public class NearRedAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain turntointake;
    public PathChain gointake;
    public PathChain gotoshootagain;
    public PathChain turntointakeagain;
    public PathChain gointakeagain;
    public PathChain gotoshootthirdtime;
    public PathChain turntointakethirdtime;
    public PathChain gointakethirdtime;
    public PathChain gotoshootfourthtime;
    public PathChain leave;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        movetoshoot = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(123.2,123.2), new Pose(84,84)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(40))
                .addParametricCallback(0.5, () -> {
                    shooter.setShooterRpm(ShooterSubsystem.ShooterMode.NEAR_SHOT);
                })
                .build();
        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(84, 82))
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(180))

                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84, 82), new Pose(123, 78))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(0.75);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();
        gotoshootagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123, 78), new Pose(84.000, 84))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.3, () -> {
                    shooter.setShooterRpm(ShooterSubsystem.ShooterMode.NEAR_SHOT);
                })
                .build();
        turntointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(84.000, 60))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();

        gointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 60), new Pose(123, 56))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();

        gotoshootthirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123, 56), new Pose(84.000, 84.000))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.3, () -> {
                    shooter.setShooterRpm(ShooterSubsystem.ShooterMode.NEAR_SHOT);
                })
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .build();
        turntointakethirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(84.000, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();

        gointakethirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 35), new Pose(123, 35))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();

        gotoshootfourthtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123, 35), new Pose(84.000, 84.000))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.3, () -> {
                    shooter.setShooterRpm(ShooterSubsystem.ShooterMode.NEAR_SHOT);
                })
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .build();
        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(105, 55.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();
        follower.setStartingPose(new Pose(123.2, 123.2, Math.toRadians(36)));

    }



    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShootServoPosition(0.12)),
                drive.FollowPath(movetoshoot, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointake, true),
                drive.FollowPath(gointake, true),
                new WaitCommand(1000),
                drive.FollowPath(gotoshootagain, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointakeagain, true),
                drive.FollowPath(gointakeagain, true),
                drive.FollowPath(gotoshootthirdtime, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointakethirdtime, true),
                drive.FollowPath(gointakethirdtime, true),
                drive.FollowPath(gotoshootfourthtime, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(leave, true)
        );
    }
}
