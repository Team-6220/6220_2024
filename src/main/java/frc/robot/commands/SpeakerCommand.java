package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class SpeakerCommand extends Command{
    private final Swerve swerve;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final DoubleSupplier translation, strafe, rotation;
    private final BooleanSupplier manualOverride;
    private double[] velocities  = {0, 0};
    private double armAngle = ArmConstants.restingSetpoint;
    public SpeakerCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier override){
        this.swerve = swerve;
        this.shooter = ShooterSubsystem.getInstance();
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.strafe = strafeSup;
        this.translation = translationSup;
        this.rotation = rotationSup;
        this.manualOverride = override;
        addRequirements(this.swerve, arm, intake, shooter);
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), OIConstants.kDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), OIConstants.kDeadband);
        swerve.setAutoTurnHeading(swerve.getHeadingToSpeaker());
        double rotationVal = swerve.getTurnPidSpeed();
        if(manualOverride.getAsBoolean()){
            rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), OIConstants.kDeadband) * SwerveConstants.maxAngularVelocity;
        }
        int invert = (Constants.isRed) ? -1 : 1; 
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed * invert), 
                rotationVal, 
                true, 
                true
        );
        // get shooter velocities
        shooter.spinToVelocity(velocities);
        // get arm angle
        arm.driveToGoal(armAngle);
        if(shooter.isAtSetpoint() && (swerve.isFacingTurnTarget() || manualOverride.getAsBoolean()) && arm.isAtGoal()){
            //call intake method to feed wheel
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
        shooter.stop();
    }

}
