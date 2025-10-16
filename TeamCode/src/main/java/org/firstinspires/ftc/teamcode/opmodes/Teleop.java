package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp (name = "TeleOp")
public class Teleop extends StealthOpMode {


    DriveSubsystem drive;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {

        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        register(drive);
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));
    }


}
