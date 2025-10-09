package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;

public class CoralDefaultCommand extends Command{
    public CoralDefaultCommand(Coral coral){

        addRequirements(coral);

    }
}