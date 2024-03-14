// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCmd;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class ListOfAllAutos {
    public static ArrayList<Pair<String, Command>> allAutos = new ArrayList<Pair<String, Command>>();
    public ListOfAllAutos()
    {
        
    }
    public static void addNewAuto(String commandName, SequentialCommandGroup newAuto)
    {
        allAutos.add(Pair.of(commandName, newAuto));
    }
    public static String getAutoName(int index)
    {
        return allAutos.get(index).getFirst();
    }

    public static Command getAutoCommand(int index)
    {
        return allAutos.get(index).getSecond();
    }
    public static int getTotalAutoNumbers()
    {
        if(!allAutos.isEmpty())
        {
            System.out.println("get sieze");
            return allAutos.size();
        }
        return -1;
    }
}
