package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RaiseShooter extends CommandBase{

    private final ShooterSubsystem shooterSubsystem;

    public RaiseShooter(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){
        shooterSubsystem.raiseShooter();
    }

    @Override
    public boolean isFinished(){
        return this.shooterSubsystem.getUpperLimitSwitch();
    }

    @Override
    public void end(boolean interrupted){
        this.shooterSubsystem.stopRotation();
    }

}
