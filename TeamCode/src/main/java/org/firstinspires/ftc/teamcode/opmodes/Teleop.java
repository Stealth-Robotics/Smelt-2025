package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.BeltCommand;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.function.DoubleSupplier;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    Follower follower;

    DriveSubsystem drive;
    BeltSubsystem beltSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        beltSubsystem = new BeltSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap, beltSubsystem, intakeSubsystem);

        /*
         Prepare the shooter for the match by setting the shooter to its idle velocity and the shot blocker to block artifacts
         from entering the shooter before it's ready
         */
        shooterSubsystem.preventShooting().schedule();
        shooterSubsystem.idle().schedule();

        drive.setDefaultCommand(drive.driveTeleop(
                () -> driveGamepad.getLeftY(),
                () -> driveGamepad.getLeftX(),
                () -> driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get()));

        shooterSubsystem.resetVelocityReaders();

        // Set our alliance based off the teleop name
        drive.setAlliance(Alliance.get() == Alliance.BLUE);

        register(drive);

        configureBindings();
    }

    private void configureBindings() {
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> shooterSubsystem.setBlockerUp());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shooterSubsystem.setBlockerDown());

        driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(drive.swapDriveMode());

        DoubleSupplier intakeSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1 : 0));
        Trigger intake = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)));
        intake.whenActive(() -> intakeSubsystem.setPower(intakeSupplier.getAsDouble()));
        intake.whenInactive(() -> intakeSubsystem.stop());

        driveGamepad.getGamepadButton((GamepadKeys.Button.DPAD_RIGHT)).whenPressed(shooterSubsystem.shootFar());
        driveGamepad.getGamepadButton((GamepadKeys.Button.DPAD_LEFT)).whenPressed(shooterSubsystem.shootNear());

        operatorGamepad.getGamepadButton((GamepadKeys.Button.X)).whenPressed(shooterSubsystem.cycle());

        //Shooter forceIdle
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A).whenActive(shooterSubsystem.forceIdle());

        Trigger shooterReverse = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP));
        shooterReverse.whenActive(shooterSubsystem.reverse());
        shooterReverse.whenInactive(shooterSubsystem.idle());

        beltSubsystem.setDefaultCommand(new BeltCommand(beltSubsystem, () -> driveGamepad.getRightY()));

        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> drive.resetHeading());
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Red Teleop", group = "Red")
    public static class RedTeleop extends Teleop { }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop { }
}
