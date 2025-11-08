package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;


@Autonomous(name = "TestAuto", group="Test", preselectTeleOp = "Teleop")
public class TestAuto extends StealthOpMode{
    DriveSubsystem driveSubsystem;
    ShooterSubsystem shooterSubsystem;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;
    private double top_pos = 0.8;
    private double bottom_pos = 0.3;
    static PathChain Path1;
    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        /*Path1 = driveSubsystem.getFollower().pathBuilder()
                    .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();*/
    }
    
    protected SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> shooterSubsystem.setPosition(bottom_pos)),
            new InstantCommand(() -> shooterSubsystem.setRpm(3200)),
            new WaitCommand(2000),
            new InstantCommand(() ->beltSubsystem.setPower(0.75)), new WaitCommand(6000),
            new InstantCommand(() -> intakeSubsystem.setPower(1))
            );
    }

    @Override
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
                shoot());
                //new WaitCommand(5000),
                //driveSubsystem.FollowPath(Path1, true)
                //);
    }
}
