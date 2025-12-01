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
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = shooter.getIntakeSubsystem();
        belt = shooter.getBeltSubsystem();
        drive.resetHeading();
    }


}
