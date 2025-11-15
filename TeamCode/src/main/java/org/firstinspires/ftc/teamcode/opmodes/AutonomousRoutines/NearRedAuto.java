package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NearRedAuto", group = "Autos", preselectTeleOp = "Teleop")
public class NearRedAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain turntointake;
    public PathChain gointake;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        movetoshoot = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(123.2,123.2), new Pose(84,84)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();
        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(100.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-180))

                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 84.000), new Pose(125, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();
        follower.setStartingPose(new Pose(123.2, 123.2, Math.toRadians(36)));

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
