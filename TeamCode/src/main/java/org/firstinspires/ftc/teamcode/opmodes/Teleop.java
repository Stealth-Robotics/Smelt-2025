package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.beltCommand;
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
    private double top_pos = 0;
    private double far_shot_pos = 0.24;
    private double bottom_pos = 0.65;
    private double far_shot_rpm = 3500;

    private double near_shot_rpm = 2600;
    private double offset;
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
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get()));

        configureBindings();
    }

    protected SequentialCommandGroup shoot() {
        return null;
    }
    private void configureBindings() {

        //driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterSubsystem.setPower(1));
        //driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> shooterSubsystem.stop());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> shooterSubsystem.changePositionUp());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> shooterSubsystem.changePositionDown());

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> drive.setAlliance(true));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> drive.setAlliance(false));


        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterSubsystem.setPosition(bottom_pos));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> shooterSubsystem.setPosition(far_shot_pos));

        driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(drive.swapDriveMode());

        DoubleSupplier intakeSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1 : 0));
        Trigger intake = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)));
        intake.whenActive(() -> intakeSubsystem.setPower(intakeSupplier.getAsDouble()));
        intake.whenInactive(() -> intakeSubsystem.stop());

        DoubleSupplier shooterFarSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0) - (operatorGamepad.getButton(GamepadKeys.Button.A) ? 1 : 0));
        Trigger shooterFar = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP)).or(new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A))));
        shooterFar.whenActive(() -> shooterSubsystem.setRpm(shooterFarSupplier.getAsDouble() * far_shot_rpm));
        shooterFar.whenInactive(() -> shooterSubsystem.setRpm(0));


        DoubleSupplier shooterNearSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0));
        Trigger shooterNear = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP)));
        shooterNear.whenActive(() -> shooterSubsystem.setRpm(shooterNearSupplier.getAsDouble() * near_shot_rpm));
        shooterNear.whenInactive(() -> shooterSubsystem.setRpm(0));


        beltSubsystem.setDefaultCommand(new beltCommand(beltSubsystem, () -> driveGamepad.getRightY()));

        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> drive.resetHeading());
    }

}
