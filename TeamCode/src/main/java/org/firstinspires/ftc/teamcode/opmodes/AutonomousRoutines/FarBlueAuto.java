package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FarBlueAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarBlueAuto extends DecodeAutos {
    public PathChain GoToShoot;
    public PathChain GoToIntake;
    public PathChain Intake;
    public PathChain GoToShoot2;
    public PathChain GoOffLine;

    @Override
    public void initialize() {
        super.initialize();
        Follower follower = drive.getFollower();
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
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(.2);
                    intake.start();
                    belt.start();
                })
                .addParametricCallback(1, () -> {
                    follower.setMaxPower(1);
                    intake.stop();
                    belt.stop();
                })
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

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                drive.FollowPath(GoToShoot, true),
                shoot(),
                drive.FollowPath(GoToIntake, true),
                drive.FollowPath(Intake, true),
                drive.FollowPath(GoToShoot2, true),
                drive.FollowPath(GoOffLine, true)
        );
    }
    //new WaitCommand(5000),
    //driveSubsystem.FollowPath(Path1, true)
    //);
}
