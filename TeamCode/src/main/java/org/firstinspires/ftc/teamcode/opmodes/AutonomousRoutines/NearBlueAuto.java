package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NearBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class NearBlueAuto extends DecodeAutos {
    public PathChain GoToShoot;
    public PathChain GoToIntake;
    public PathChain Intake;
    public PathChain GoToShoot2;

    @Override
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        GoToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.000, 121.800), new Pose(45.300, 108.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(140))
                .build();

        GoToIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.300, 108.100), new Pose(39.100, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0))
                .build();

        Intake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(39.100, 84.000), new Pose(13.400, 84.000))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        GoToShoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.400, 84.000), new Pose(47.400, 115.900))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(150))
                .build();

    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(GoToShoot, true),
                shoot(),
                drive.FollowPath(GoToIntake, true),
                drive.FollowPath(Intake, true),
                drive.FollowPath(GoToShoot2, true)
        );
    }
}
