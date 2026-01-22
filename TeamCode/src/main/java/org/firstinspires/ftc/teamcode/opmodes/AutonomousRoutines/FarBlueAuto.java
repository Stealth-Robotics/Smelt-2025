package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FarBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarBlueAuto extends DecodeAutos{
    public PathChain movetoshoot;
    public PathChain gointake;
    public PathChain keepintaking;
    public PathChain gotoshootagain;
    public PathChain leave;
    private double offset = 0.2;
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        movetoshoot = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(56, 12)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        gointake = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 12),

                                new Pose(14, 10)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();

        keepintaking = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(10.000, 10.000),
                                new Pose(4.576, 8.676),
                                new Pose(10.000, 20.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))

                .build();

        gotoshootagain = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 20.000),

                                new Pose(56.000, 10.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(110))
                .addParametricCallback(0.5, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(1);
                    intake.setPower(0.5);
                    belt.setPower(0);
                })
                .build();
        leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 10.000),

                                new Pose(36.000, 10.000)
                        )
                ).setTangentHeadingInterpolation()
                .addParametricCallback(0.5, () -> {
                    shooter.setRpm(0);
                    shooter.setShootServoPosition(0.12);
                    intake.setPower(0);
                    belt.setPower(0);
                })
                .build();


        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));
    }




    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShootServoPosition(0.12)),
                drive.FollowPath(movetoshoot, true),
                shooter.shootThreeBallsFar(),
                drive.FollowPath(gointake, true),
                drive.FollowPath(keepintaking, true),
                drive.FollowPath(gotoshootagain, true),
                shooter.shootThreeBallsFar(),
                drive.FollowPath(leave, true)
        );
    }
}
