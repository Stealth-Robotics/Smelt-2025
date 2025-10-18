package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class shootCommand extends CommandBase {
    public shootCommand(ShooterSubsystem shooter){
        shooter.setPower(1);
    }
}
