package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FarBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarBlueAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain leave;
    private double offset = 0.2;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        movetoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 9), new Pose(56.000, 10))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));
        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 10), new Pose(40, 10))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));

    }




    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShootServoPosition(0.12)),
                drive.FollowPath(movetoshoot, true),
                new WaitUntilCommand(() -> drive.doAimAtTarget(.1, offset, 50)),
                shooter.shootThreeBallsFar(),
                drive.FollowPath(leave, true)

        );
    }
}
