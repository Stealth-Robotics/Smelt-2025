package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NearBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class NearBlueAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain turntointake;
    public PathChain gointake;
    public PathChain goback;
    public PathChain gohitgate;
    public PathChain gotoshootagain;
    public PathChain turntointakeagain;
    public PathChain gointakeagain;
    public PathChain gobackagain;
    public PathChain gotoshootthirdtime;
    public PathChain turntointakethirdtime;
    public PathChain gointakethirdtime;
    public PathChain gotoshootfourthtime;
    public PathChain leave;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();

        movetoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.800, 123.200), new Pose(57, 88))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(137))
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(2650);
                })
                .build();

        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 88), new Pose(50.000, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(0))
                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50.000, 85), new Pose(27, 85))
                )
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        goback = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27, 85), new Pose(30, 85))
                )
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        gohitgate = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30, 85), new Pose(20, 78))
                )
                .addParametricCallback(0, () -> {
                    intake.setPower(1);
                    follower.setMaxPower(0.5);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gotoshootagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20, 78), new Pose(57, 90))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(139))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(2650);
                })

                .build();

        turntointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 90), new Pose(54, 61))
                )
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(0))
                .build();

        gointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54, 61), new Pose(15, 61))
                )
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        gobackagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(15, 61), new Pose(30, 61))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        gotoshootthirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30, 61), new Pose(57, 90))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(2650);
                })
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(139))
                .build();
        turntointakethirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 90.000), new Pose(54.000, 43))
                )
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(0))
                .build();

        gointakethirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(54.000, 43), new Pose(16, 43)))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gotoshootfourthtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16, 43), new Pose(57, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(139))
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(2650);
                })
                .build();
        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57, 90), new Pose(45, 63))
                )
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))
                .build();
        follower.setStartingPose(new Pose(20.8, 123.2, Math.toRadians(144)));
    }



    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShootServoPosition(0.12)),
                drive.FollowPath(movetoshoot, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointake, true),
                drive.FollowPath(gointake, true),
                drive.FollowPath(goback, true),
                drive.FollowPath(gohitgate, true),
                new WaitCommand(500),
                drive.FollowPath(gotoshootagain, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointakeagain, true),
                drive.FollowPath(gointakeagain, true),
                drive.FollowPath(gobackagain, true),
                drive.FollowPath(gotoshootthirdtime, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointakethirdtime, true),
                drive. FollowPath(gointakethirdtime, true),
                drive.FollowPath(gotoshootfourthtime, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(leave, true)
        );
    }
}

