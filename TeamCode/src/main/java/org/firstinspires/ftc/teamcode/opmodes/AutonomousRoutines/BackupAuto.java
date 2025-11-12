package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BackupAuto extends DecodeAutos{
    public PathChain move;

    @Override
    public SequentialCommandGroup initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        move = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 9.000), new Pose(56.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        return null;
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(move, true)
        );
    }
}
