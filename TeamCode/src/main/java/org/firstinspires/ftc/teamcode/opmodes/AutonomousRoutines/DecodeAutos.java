package org.firstinspires.ftc.teamcode.opmodes.AutonomousRoutines;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class DecodeAutos extends StealthOpMode {
    protected BeltSubsystem belt;
    protected CameraSubsystem camera;
    protected DriveSubsystem drive;
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;



    @Override
    public void initialize() {
        belt = new BeltSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

    }

    protected SequentialCommandGroup shoot() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setHoodDown()),
                new InstantCommand(() -> shooter.setRpm(3500)),
                new WaitCommand(2000),
                new InstantCommand(() ->belt.setPower(0.75)), new WaitCommand(6000),
                new InstantCommand(() -> intake.setPower(1))
        );
    }
}
