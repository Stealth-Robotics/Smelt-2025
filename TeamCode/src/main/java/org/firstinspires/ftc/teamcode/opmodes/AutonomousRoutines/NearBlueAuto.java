package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
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
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();

        movetoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.800, 123.200), new Pose(60.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();

        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(44.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 84.000), new Pose(19.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();

        follower.setStartingPose(new Pose(20.8, 123.2, Math.toRadians(144)));
    }



    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(movetoshoot, true),
                shooter.shootThreeBallsNear(),
                drive.FollowPath(turntointake, true),
                drive.FollowPath(gointake, true)
        );
    }
}

