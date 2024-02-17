package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
        s_Swerve.setAutoTurnHeading(headingTarget.getAsDouble());
        SmartDashboard.putNumber("headingTarget", headingTarget.getAsDouble());
        double rotationVal = s_Swerve.getTurnPidSpeed();
        int invert =  (Constants.isRed) ? -1 : 1; 
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed * invert), 
                rotationVal, 
                true, 
                true
            );
    }
}
