package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends Command{
    private final ArmSubsystem armSubsystem;

    private final Supplier<Boolean> aButton, yButton, rBumper, lBumper;

    private final Supplier<Double> joystick;

    public ArmTestCommand(Supplier<Boolean> aB, Supplier<Boolean> yB, Supplier<Boolean> lBump,Supplier<Boolean> rBump, Supplier<Double> js){
        this.armSubsystem = ArmSubsystem.getInstance();

        this.aButton = aB;
        this.yButton = yB;
        this.rBumper = rBump;
        this.lBumper = lBump;
        this.joystick = js;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        boolean pid = false;
        double setpoint = 0, raw_value = 0;
        if(this.aButton.get()){
            pid = true;
            setpoint = 0;
        }else if(this.yButton.get()){
            pid = true;
            setpoint = 85;
        }else if(this.rBumper.get()){
            pid = true;
            setpoint = 60;
        }else if(this.lBumper.get()){
            pid = true;
            setpoint = 40;
        }else if(Math.abs(this.joystick.get()) > .05){
            // pid = false;
            raw_value = this.joystick.get() / 2;
        }
        if(pid){
            raw_value = armSubsystem.calculate(setpoint);
            System.out.println("succcesss!!!!");
        }
        
        armSubsystem.drive(raw_value);
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.drive(0);
    }

}
