package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import android.graphics.Point;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BackupAuto extends DecodeAutos{
    public PathChain move;
    static Pose startPose = new Pose(56, 9, 0);
    static Pose endPose = new Pose(56, 36, 0);
    @Override
    public SequentialCommandGroup initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        move = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56, 9), new Pose(56, 36))
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        return null;
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(move, true)
        );
    }
}
