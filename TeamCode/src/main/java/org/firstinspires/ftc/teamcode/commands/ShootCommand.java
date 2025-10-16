package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {
    public ShootCommand(ShooterSubsystem shooter){
        shooter.setPower(1);
    }
}
