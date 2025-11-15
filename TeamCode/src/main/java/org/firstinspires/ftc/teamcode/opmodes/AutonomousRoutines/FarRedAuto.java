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

@Autonomous(name = "FarRedAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarRedAuto extends DecodeAutos{
    public PathChain movetointake;
    public PathChain gointake;
    public PathChain gobacktoshoot;
    private double offset = -2.5;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        movetointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.000, 9.000), new Pose(106.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(106.000, 35.000), new Pose(130.000, 35.000))
                )
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        gobacktoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(86, 14))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(60))
                .build();

        follower.setStartingPose(new Pose(84, 9, Math.toRadians(90)));
    }




    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> drive.doAimAtTarget(.1,  offset,50)),
                shooter.shootThreeBallsFar(),
                drive.FollowPath(movetointake, true),
                drive.FollowPath(gointake, true),
                drive.FollowPath(gobacktoshoot, true)
        );
    }
}
