package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.blinkin;

public class IntakeCommand extends Command{

    private final PIDController limelightPidController;
    private final Swerve swerve;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final PhotonVisionSubsystem vis;
    private final blinkin s_Blinkin;
    private final BooleanSupplier autoControl;
    private double armAngle = ArmConstants.hoverSetpoint;
    private XboxController driver;
    private boolean isAuto;
    private int timeWithoutTarget = 0, stopIntakeDelay = 20;

    private final TunableNumber turnkP = new TunableNumber("intakeTurnkP", 4);
    private final TunableNumber turnkD = new TunableNumber("intakeTurnkD", 0.002);
    private final TunableNumber turnkI = new TunableNumber("intakeTurnkI", 0);
    private final TunableNumber turnTolerance = new TunableNumber("turnTolerance", 3);

    public IntakeCommand(Swerve swerve, XboxController driver, BooleanSupplier autoControl) {
        isAuto = false;
        this.swerve = swerve;
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.vis = PhotonVisionSubsystem.getInstance();
        this.s_Blinkin = blinkin.getInstance();
        this.autoControl = autoControl;
        this.driver = driver;
        limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
        limelightPidController.setTolerance(turnTolerance.get());
        limelightPidController.setIZone(.5);
        timeWithoutTarget = 0;
        addRequirements(this.swerve, arm, intake, vis);
    }

    public IntakeCommand(Swerve swerve) {
        isAuto = true;
        timeWithoutTarget = 0;
        this.swerve = swerve;
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        this.vis = PhotonVisionSubsystem.getInstance();
        this.s_Blinkin = blinkin.getInstance();
        this.autoControl = ()-> true;
        this.driver = null;
        limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
        limelightPidController.setTolerance(turnTolerance.get());
        limelightPidController.setIZone(.5);
        addRequirements(this.swerve, arm, intake, vis);
    }

    @Override
    public void initialize() {
        timeWithoutTarget = 0;
    }

    @Override
    public void execute(){
        double[] driverInputs;
        double rotationVal = 0, translation = 0, strafeVal = 0;
        if(!isAuto) {
            driverInputs = OIConstants.getDriverInputs(driver);
            translation = driverInputs[0];
            strafeVal = driverInputs[1];
            rotationVal = driverInputs[2];
        }

        if(autoControl.getAsBoolean()) {
            if(vis.getHasTargets()) {
                timeWithoutTarget = 0;
                rotationVal = limelightPidController.calculate(vis.getTurnOffset());

                rotationVal = (rotationVal > SwerveConstants.maxAngularVelocity)?SwerveConstants.maxAngularVelocity:(rotationVal< -SwerveConstants.maxAngularVelocity)?-SwerveConstants.maxAngularVelocity:rotationVal;
                // System.out.println("success!");
                if(Math.abs(vis.getTurnOffset()) < turnTolerance.get()) {
                    rotationVal = 0;
                    s_Blinkin.solid_blue();
                } else {
                    s_Blinkin.solid_red();
                }
                translation = -.3 * SwerveConstants.maxSpeed;
            }
            else
            {
                timeWithoutTarget++;
            }

        } else {
            s_Blinkin.solid_gold();
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
    public boolean isFinished() {
        if(intake.noteInBeam() || (timeWithoutTarget > stopIntakeDelay && isAuto)) {
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
    }
}
