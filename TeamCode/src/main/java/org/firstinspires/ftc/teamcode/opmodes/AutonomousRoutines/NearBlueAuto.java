package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NearBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class NearBlueAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain turntointake;
    public PathChain gointake;
    public PathChain gotoshootagain;
    public PathChain turntointakeagain;
    public PathChain gointakeagain;
    public PathChain gotoshootthirdtime;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();

        movetoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.800, 123.200), new Pose(58, 90))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(141))
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(2600);
                })
                .build();

        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.000, 90), new Pose(50.000, 103))
                )
                .setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(0))
                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(50.000, 103), new Pose(30, 103))
                )
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gotoshootagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(26, 103), new Pose(58, 90))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(141))
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.3, () -> {
                    shooter.setRpm(2600);
                })

                .build();

        turntointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.000, 85.000), new Pose(58.000, 78))
                )
                .setLinearHeadingInterpolation(Math.toRadians(141), Math.toRadians(0))
                .build();

        gointakeagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(58.000, 78), new Pose(25, 78))
                )
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        gotoshootthirdtime = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(30.000, 78), new Pose(58.000, 90))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    intake.setPower(0.3);
                })
                .addParametricCallback(0.3, () -> {
                    shooter.setRpm(2600);
                })
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(141))
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
                drive.FollowPath(gotoshootagain, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointakeagain, true),
                drive.FollowPath(gointakeagain, true),
                drive.FollowPath(gotoshootthirdtime, true),
                shooter.shootThreeBallsNear()
        );
    }
}

