package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand (IntakeSubsystem intake){
        addCommands(
                new InstantCommand(()->intake.setPower(1))
        );
    }
}
