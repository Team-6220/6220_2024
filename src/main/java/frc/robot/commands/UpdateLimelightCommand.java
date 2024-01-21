package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class UpdateLimelightCommand extends Command {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    ShuffleboardTab tab = Shuffleboard.getTab("Limelight"); 

    @Override
    public void initialize() {
        double tx = table.getEntry("tx").getDouble(0.0);
         System.out.println(tx);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
