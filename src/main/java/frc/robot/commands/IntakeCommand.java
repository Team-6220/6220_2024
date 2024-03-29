package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;

public class IntakeCommand extends Command{

    private final PIDController limelightPidController;
    private final Swerve swerve;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final PhotonVisionSubsystem vis;
    private final BooleanSupplier manualOverride;
    private double armAngle = ArmConstants.hoverSetpoint;
    private XboxController driver;

    private final TunableNumber turnkP = new TunableNumber("intakeTurnkP", 4);
    private final TunableNumber turnkD = new TunableNumber("intakeTurnkD", 0.002);
    private final TunableNumber turnkI = new TunableNumber("intakeTurnkI", 0);
    private final TunableNumber turnTolerance = new TunableNumber("turnTolerance", 3);

    public IntakeCommand(Swerve swerve, XboxController driver, BooleanSupplier override) {
        this.swerve = swerve;
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.vis = PhotonVisionSubsystem.getInstance();
        this.manualOverride = override;
        this.driver = driver;
        limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
        limelightPidController.setTolerance(turnTolerance.get());
        limelightPidController.setIZone(4);
        addRequirements(this.swerve, arm, intake, vis);
    }

    @Override
    public void execute(){

        double[] driverInputs = OIConstants.getDriverInputs(driver);

        double rotationVal = driverInputs[2];
        double translation = driverInputs[0];
        double strafeVal = driverInputs[1];

        if(manualOverride.getAsBoolean()) {
            if(vis.getHasTargets()) {
                rotationVal = limelightPidController.calculate(vis.getTurnOffset());

                rotationVal = (rotationVal > SwerveConstants.maxAngularVelocity)?SwerveConstants.maxAngularVelocity:(rotationVal< -SwerveConstants.maxAngularVelocity)?-SwerveConstants.maxAngularVelocity:rotationVal;
                // System.out.println("success!");
                if(Math.abs(vis.getTurnOffset()) < turnTolerance.get()) {
                    rotationVal = 0;
                }
                translation = -.3 * SwerveConstants.maxSpeed;
            }
        }




        swerve.drive(
                new Translation2d(translation, strafeVal), 
                rotationVal, 
                false, 
                true
        );

        intake.driveToIntake();

        arm.driveToGoal(ArmConstants.intakeSetpoint);

    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
    }
}
