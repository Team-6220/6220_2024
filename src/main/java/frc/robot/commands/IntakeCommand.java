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
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;

public class IntakeCommand extends Command{
    private final Swerve swerve;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final PhotonVisionSubsystem vis;
    private final DoubleSupplier translation, strafe, rotation;
    private final BooleanSupplier manualOverride, trigger;
    private double armAngle = ArmConstants.hoverSetpoint;
    public IntakeCommand(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier override, BooleanSupplier trigger){
        this.swerve = swerve;
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.vis = PhotonVisionSubsystem.getInstance();
        this.strafe = strafeSup;
        this.translation = translationSup;
        this.rotation = rotationSup;
        this.manualOverride = override;
        this.trigger = trigger;
        addRequirements(this.swerve, arm, intake, vis);
    }

    @Override
    public void execute(){
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), OIConstants.kDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), OIConstants.kDeadband);
        double headingTarget = swerve.getHeadingDegrees();
        swerve.setAutoTurnHeading(headingTarget);
        double rotationVal = swerve.getTurnPidSpeed();
        if(!vis.getHasTargets() || manualOverride.getAsBoolean()){
            rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), OIConstants.kDeadband) * SwerveConstants.maxAngularVelocity;
        }else if(vis.getHasTargets() && !manualOverride.getAsBoolean()){
            headingTarget += vis.getTurnOffset();
            swerve.setAutoTurnHeading(headingTarget);
            rotationVal = swerve.getTurnPidSpeed();
        }
        int invert = (Constants.isRed) ? -1 : 1; 
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed * invert), 
                rotationVal, 
                true, 
                true
        );
        if(trigger.getAsBoolean()){
            armAngle = ArmConstants.intakeSetpoint;
            intake.driveToIntake();
        }
        else{
            armAngle = ArmConstants.hoverSetpoint;
            intake.stop();
        }
        arm.driveToGoal(armAngle);
        if((swerve.isFacingTurnTarget() || manualOverride.getAsBoolean()) && arm.isAtGoal()){
            //call intake method to feed wheel
        }
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
    }
}
