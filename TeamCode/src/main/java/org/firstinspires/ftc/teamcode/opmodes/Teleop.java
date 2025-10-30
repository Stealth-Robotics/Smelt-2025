package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.shootCommand;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@TeleOp (name = "TeleOp")
public class Teleop extends StealthOpMode {

    DriveSubsystem drive;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;
    Follower follower;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {

        //beltSubsystem = new BeltSubsystem(hardwareMap);
        //intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);


        drive = new DriveSubsystem(hardwareMap, telemetry);


        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        register(drive);

        telemetry.addData("GamepadRaw X", driveGamepad.getLeftX());
        //negative for strafing because some motors are reversed (testbot)
        drive.setDefaultCommand(drive.driveTeleop(
                () -> driveGamepad.getLeftY(),
                () -> driveGamepad.getRightX(),
                () -> driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).get(),
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get()));
        configureBindings();
    }


    private void configureBindings() {

        //driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new beltCommand(beltSubsystem));
        //driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new hoodCommand(shooterSubsystem));
        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterSubsystem.setPower(1));
        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> shooterSubsystem.stop());
        //driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand(intakeSubsystem));
        //driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(()-> drive.resetHeading());
    }

}
