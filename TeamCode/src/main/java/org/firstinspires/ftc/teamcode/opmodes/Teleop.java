package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.beltCommand;
import org.firstinspires.ftc.teamcode.commands.shootCommand;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.function.DoubleSupplier;

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

        beltSubsystem = new BeltSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);


        drive = new DriveSubsystem(hardwareMap, telemetry);


        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        register(drive);

        telemetry.addData("GamepadRaw X", driveGamepad.getLeftX());
        //negative for strafing because some motors are reversed (testbot)
        drive.setDefaultCommand(drive.driveTeleop(
                () -> driveGamepad.getLeftY(),
                () -> driveGamepad.getLeftX(),
                () -> driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).get(),
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get()));
        configureBindings();
    }


    private void configureBindings() {

        //driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterSubsystem.setPower(1));
        //driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> shooterSubsystem.stop());

        DoubleSupplier intakeSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1 : 0));
        Trigger intake = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)));
        intake.whenActive(() -> intakeSubsystem.setPower(intakeSupplier.getAsDouble()));
        intake.whenInactive(() -> intakeSubsystem.stop());

        DoubleSupplier shooterSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.B) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.Y) ? 1 : 0));
        Trigger shooter = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.B)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.Y)));
        shooter.whenActive(() -> shooterSubsystem.setPower(shooterSupplier.getAsDouble()));
        shooter.whenInactive(() -> shooterSubsystem.stop());

        beltSubsystem.setDefaultCommand(new beltCommand(beltSubsystem, () -> driveGamepad.getRightY()));

        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> drive.resetHeading());
    }

}
