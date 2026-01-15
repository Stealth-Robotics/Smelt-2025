package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
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
    private double far_shot_pos = 0.27;
    private double bottom_pos = 0.65;
    private double far_shot_rpm = 3600;
    private double near_shot_rpm = 2650;
    private double cycle_rpm = 500;
    private double reverse_rpm = -1500;
    private double offset;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        intakeSubsystem = shooterSubsystem.getIntakeSubsystem();
        beltSubsystem = shooterSubsystem.getBeltSubsystem();

        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        shooterSubsystem.setShootServoPosition(0.12);
        shooterSubsystem.setRpm(500);
        shooterSubsystem.setLedColor(0.28);
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


        operatorGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> shooterSubsystem.hoodSetPosition(bottom_pos));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> shooterSubsystem.hoodSetPosition(far_shot_pos));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> shooterSubsystem.setBlockerUp());
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> shooterSubsystem.setBlockerDown());


        driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(drive.swapDriveMode());

        DoubleSupplier intakeSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) ? 1 : 0));
        Trigger intake = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)));
        intake.whenActive(() -> intakeSubsystem.setPower(intakeSupplier.getAsDouble()));
        intake.whenInactive(() -> intakeSubsystem.stop());

        driveGamepad.getGamepadButton((GamepadKeys.Button.DPAD_RIGHT)).whenPressed(shooterSubsystem.setRpmFar());
        driveGamepad.getGamepadButton((GamepadKeys.Button.DPAD_LEFT)).whenPressed(shooterSubsystem.setRpmNear());
        operatorGamepad.getGamepadButton((GamepadKeys.Button.X)).whenPressed(shooterSubsystem.setRpmCycle());

        Trigger shooterReverse = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP));
        shooterReverse.whenActive(shooterSubsystem.setReverseRpm(reverse_rpm));
        shooterReverse.whenInactive(shooterSubsystem.setReverseRpm(0));

//        shooterFar.whenActive(() -> shooterSubsystem.setRpm(shooterFarSupplier.getAsDouble() * far_shot_rpm));
        //shooterFar.whenInactive(() -> shooterSubsystem.setRpm(0));


//        DoubleSupplier shooterNearSupplier = () -> ((driveGamepad.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0));
//        Trigger shooterNear = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_LEFT)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP)));
//        shooterNear.whenActive(() -> shooterSubsystem.setRpm(shooterNearSupplier.getAsDouble() * near_shot_rpm));
        //shooterNear.whenInactive(() -> shooterSubsystem.setRpm(0));

//        DoubleSupplier shooterCycleSupplier = () -> ((operatorGamepad.getButton(GamepadKeys.Button.X) ? 1 : 0) - (operatorGamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON) ? 1 : 0));
//        Trigger shooterCycle = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.X)).or(new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)));
//        shooterCycle.whenActive(() -> shooterSubsystem.setRpm(shooterCycleSupplier.getAsDouble() * cycle_rpm));
        //shooterCycle.whenInactive(() -> shooterSubsystem.setRpm(0));

//        DoubleSupplier shooterCycleSupplier = () -> ((operatorGamepad.getButton(GamepadKeys.Button.X) ? 1 : 0) - (driveGamepad.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0));
//        Trigger shooterCycle = new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.X)).or(new Trigger(() -> driveGamepad.getButton(GamepadKeys.Button.DPAD_UP)));
//        shooterCycle.whenActive(() -> shooterSubsystem.setRpm(shooterCycleSupplier.getAsDouble() * cycle_rpm));
//        shooterCycle.whenInactive(() -> shooterSubsystem.setRpm(0));



        beltSubsystem.setDefaultCommand(new beltCommand(beltSubsystem, () -> driveGamepad.getRightY()));

        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(()-> drive.resetHeading());
    }

}
