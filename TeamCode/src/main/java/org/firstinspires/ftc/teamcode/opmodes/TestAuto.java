package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;


@Autonomous(name = "TestAuto")
public class TestAuto extends StealthOpMode{
    Follower follower;
    DriveSubsystem driveSubsystem;
    ShooterSubsystem shooterSubsystem;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;

    private double top_pos = 0.4;
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
    }

    @Override
    protected SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> shooterSubsystem.setRpm(3000)),
            new WaitCommand(1000),
            new InstantCommand(() ->beltSubsystem.setPower(0.75))
            );
    }
    @Override
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
                shoot()
        );
    }
}
