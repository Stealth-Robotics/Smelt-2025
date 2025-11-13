package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FarRedAuto", group = "Autos", preselectTeleOp = "Teleop")
public class FarRedAuto extends DecodeAutos {
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
                        new BezierLine(new Pose(88, 9.000), new Pose(84.700, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(74))
                .build();

        GoToIntake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.7, 18.000), new Pose(96.500, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(74), Math.toRadians(180))
                .build();

        Intake = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.500, 36.000), new Pose(130.500, 36.000))
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
                        new BezierLine(new Pose(130.500, 36.000), new Pose(84.700, 18.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(74))
                .build();

        GoOffLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(84.700, 18.000), new Pose(89.000, 60.400))
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
