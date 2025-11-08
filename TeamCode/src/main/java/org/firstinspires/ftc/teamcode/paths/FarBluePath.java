package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarBluePath {
    public PathChain GoToShoot;
    public PathChain GoToIntake;
    public PathChain Intake;
    public PathChain GoToShoot2;
    public PathChain GoOffLine;

    public FarBluePath(Follower follower) {
        GoToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 9.000), new Pose(59.300, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(106))
                .build();

        GoToIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.300, 18.000), new Pose(47.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(106), Math.toRadians(0))
                .build();

        Intake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(47.500, 36.000), new Pose(13.500, 36.000))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        GoToShoot2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(13.500, 36.000), new Pose(59.300, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(106))
                .build();

        GoOffLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(59.300, 18.000), new Pose(56.000, 60.400))
                )
                .setTangentHeadingInterpolation()
                .build();
    }
}
