package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BackupAuto", group = "Autos", preselectTeleOp = "Teleop")
public class BackupAuto extends DecodeAutos{
    public PathChain forwards;
    public void initialize() {
        super.initialize();
            Follower follower = drive.getFollower();
            forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(0,0), new Pose(0,48)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
            follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(forwards, true)
        );
    }
}
