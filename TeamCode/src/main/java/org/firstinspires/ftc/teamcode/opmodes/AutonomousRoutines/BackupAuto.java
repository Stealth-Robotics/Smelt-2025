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
    public PathChain move;
    public void initialize() {
        super.initialize();
            Follower follower = drive.getFollower();
            follower.setStartingPose(new Pose(56.000, 9, 90));
            move = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9), new Pose(56.000, 50.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(move, true)
        );
    }
}
