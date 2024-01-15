package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, tLimiter;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.xLimiter = new SlewRateLimiter(25);
        this.yLimiter = new SlewRateLimiter(25);
        this.tLimiter = new SlewRateLimiter(25);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        //You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double xSpeed = xLimiter.calculate(m_translationXSupplier.getAsDouble());
        double ySpeed = yLimiter.calculate(m_translationYSupplier.getAsDouble());
        double tSpeed = tLimiter.calculate(m_rotationSupplier.getAsDouble());
        m_drivetrainSubsystem.drive(
                Constants.m_kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        tSpeed,
                        Rotation2d.fromDegrees(m_drivetrainSubsystem.getHeading())
                )
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(
            Constants.m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0.0,
                    0.0,
                    0.0,
                    Rotation2d.fromDegrees(m_drivetrainSubsystem.getHeading())
                )
            )
        );
    }
}
