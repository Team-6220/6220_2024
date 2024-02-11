package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OIConstants.kDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OIConstants.kDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), OIConstants.kDeadband);

        //Because origin is always blue, invert the drivers x and y controls if they are on red
        var invert = DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Blue).orElse(false) ? 1: -1;

        // System.out.println(SwerveConstants.maxAngularVelocity);
        if(!s_Swerve.isAutoTurning()) {
            /* Drive */
            s_Swerve.drive(
                new Translation2d(translationVal*invert, strafeVal*invert).times(SwerveConstants.maxSpeed), 
                rotationVal * SwerveConstants.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        } else {
            s_Swerve.drive(
                new Translation2d(translationVal*invert, strafeVal*invert).times(SwerveConstants.maxSpeed), 
                s_Swerve.getTurnPidSpeed(), 
                !robotCentricSup.getAsBoolean(), 
                true
            );
            // System.out.println("LET\"S GOOOOO");
        }
        
    }
}