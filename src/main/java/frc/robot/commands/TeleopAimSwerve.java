package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class TeleopAimSwerve extends Command{
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup, strafeSup, headingTarget;

    public TeleopAimSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier headingTarget) {
        this.s_Swerve = s_Swerve;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.headingTarget = headingTarget;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), OIConstants.kDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), OIConstants.kDeadband);
        double rotationVal = s_Swerve.getTurnPidSpeed();
        s_Swerve.setTurnControllerGoal(headingTarget.getAsDouble());
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
                rotationVal * SwerveConstants.maxAngularVelocity, 
                true, 
                true
            );
    }
}
