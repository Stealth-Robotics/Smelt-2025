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

@Autonomous(name = "FarRedAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarRedAuto extends DecodeAutos {
    public PathChain gotoshoot;
    public PathChain turntointake;
    public PathChain gointake;
    public PathChain goback;
    public PathChain gotoshootagain;
    public PathChain leave;
    private double offset = 2.5;

    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
        gotoshoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 8), new Pose(88.000, 10))
                )
                .setTangentHeadingInterpolation()
                .build();
        turntointake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 15), new Pose(125.000, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        gointake = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 18.000),
                                new Pose(133.502, 20.296),
                                new Pose(134.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .addParametricCallback(0, () -> {
                    shooter.setShootServoPosition(0.12);
                    follower.setMaxPower(0.5);
                    intake.setPower(1);
                    belt.setPower(-0.75);
                })
                .build();
        goback = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 9.000), new Pose(134.000, 20))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        gotoshootagain = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.000, 15.000), new Pose(88.000, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
        leave = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 10), new Pose(108, 10))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
    }




    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(gotoshoot, true),
                new InstantCommand(() -> shooter.setShootServoPosition(0.12)),
                new InstantCommand(() -> shooter.setRpm(1000)),
                new WaitUntilCommand(() -> drive.doAimAtTarget(.1,  offset,50)),
                shooter.shootThreeBallsFar(),
                drive.FollowPath(leave, true)
//                drive.FollowPath(turntointake, true),
//                drive.FollowPath(gointake, true),
//                drive.FollowPath(goback, true),
//                drive.FollowPath(gotoshootagain, true),
//                new WaitUntilCommand(() -> drive.doAimAtTarget(.1,  offset,50)),
//                shooter.shootThreeBallsFar()
        );
    }
}
