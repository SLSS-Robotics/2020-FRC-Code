package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSubsystem;

public class ExtendSpinner extends CommandBase {
    private final ColorSubsystem colorSubsystem;

    public ExtendSpinner(ColorSubsystem colorSubsystem){
        this.colorSubsystem = colorSubsystem;
    }

    @Override
    public void initialize(){
        colorSubsystem.raiseArm();
    }

    @Override
    public boolean isFinished(){
        return colorSubsystem.getTopLimitSwitch();
    }

    @Override
    public void end(boolean interrupted){
        colorSubsystem.stopArm();
    }


}
