package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.util.RumbleManager;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.blinkin;

public class IntakeCommand extends Command{

    private final PIDController limelightPidController;
    private final Swerve swerve;
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final ShooterSubsystem shooter;
    private final PhotonVisionSubsystem vis;
    private final blinkin s_Blinkin;
    private final BooleanSupplier autoControl;
    private double armAngle = ArmConstants.hoverSetpoint;
    private XboxController driver;
    private boolean isAuto;
    private int timeWithoutTarget = 0, stopIntakeDelay = 100;
    private boolean isParallelingWithAutobuilder = false;

    private boolean isFieldRelative = false;

    private final TunableNumber turnkP = new TunableNumber("intakeTurnkP", 4);
    private final TunableNumber turnkD = new TunableNumber("intakeTurnkD", 0.002);
    private final TunableNumber turnkI = new TunableNumber("intakeTurnkI", 0);
    private final TunableNumber turnTolerance = new TunableNumber("turnTolerance", 3);

    public IntakeCommand(Swerve swerve, XboxController driver, BooleanSupplier autoControl) {
        isAuto = false;
        isParallelingWithAutobuilder = false;
        this.swerve = swerve;
        this.intake = IntakeSubsystem.getInstance();
        this.arm = ArmSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();
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

    // public IntakeCommand(Swerve swerve) {
    //     isAuto = true;
    //     isParallelingWithAutobuilder = false;
    //     swerve.setIsAuto(true);
    //     timeWithoutTarget = 0;
    //     this.swerve = swerve;
    //     this.intake = IntakeSubsystem.getInstance();
    //     this.arm = ArmSubsystem.getInstance();
    //     this.vis = PhotonVisionSubsystem.getInstance();
    //     this.s_Blinkin = blinkin.getInstance();
    //     this.autoControl = ()-> true;
    //     this.driver = null;
    //     shooter = ShooterSubsystem.getInstance();
    //     limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
    //     limelightPidController.setTolerance(turnTolerance.get());
    //     limelightPidController.setIZone(.5);
    //     addRequirements(this.swerve, arm, intake, vis);
    // }

    // public IntakeCommand(Swerve swerve, boolean isParallelingWithAutobuilder)
    // {
    //     this.isParallelingWithAutobuilder = isParallelingWithAutobuilder;
    //     shooter = ShooterSubsystem.getInstance();
    //     isAuto = true;
    //     swerve.setIsAuto(true);
    //     timeWithoutTarget = 0;
    //     this.swerve = swerve;
    //     this.intake = IntakeSubsystem.getInstance();
    //     this.arm = ArmSubsystem.getInstance();
    //     this.vis = PhotonVisionSubsystem.getInstance();
    //     this.s_Blinkin = blinkin.getInstance();
    //     this.autoControl = ()-> true;
    //     this.driver = null;
    //     limelightPidController = new PIDController(turnkP.get(),turnkI.get(),turnkD.get());
    //     limelightPidController.setTolerance(turnTolerance.get());
    //     limelightPidController.setIZone(.5);
    //     addRequirements(arm, intake, vis);
    // }

    @Override
    public void initialize() {
        timeWithoutTarget = 0;
    }

    @Override
    public void execute(){
        intake.feedIntake();
        arm.driveToGoal(ArmConstants.intakeSetpoint);
        double[] driverInputs;
        double rotationVal = 0, translation = 0, strafeVal = 0;
        if(!isAuto && !autoControl.getAsBoolean()) {
            driverInputs = OIConstants.getDriverInputs(driver);
            translation = driverInputs[0];
            strafeVal = driverInputs[1];
            rotationVal = driverInputs[2];
            isFieldRelative = true;
        }

        if((!isAuto && autoControl.getAsBoolean() && !isParallelingWithAutobuilder)|| isAuto) { //you have to hold it to fire the auto aim
            // System.out.println("ISAUTO" + isAuto);
            if(vis.getHasTargets() && arm.isAtGoal()) {
                // System.out.println("let's see,,,");
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
                translation = -.5 * SwerveConstants.maxSpeed;
                isFieldRelative = false;
            }
            else
            {
                isFieldRelative = true;
                timeWithoutTarget++;
            }

        }
        else 
        {
            s_Blinkin.solid_gold();
        }


        if(((isAuto && !swerve.getIsAutoOverShoot()) && vis.getHasTargets()) ||( !isAuto && !isParallelingWithAutobuilder && arm.getArmPosition() > 80))
        {
            swerve.drive(
                    new Translation2d(translation, strafeVal), 
                    rotationVal, 
                    isFieldRelative, 
                    true
            );
        } 
        else if(swerve.getIsAutoOverShoot())
        {
            swerve.stopDriving();
        }
        else {
            swerve.drive(
                new Translation2d(),0,isFieldRelative,true
            );
        }
        
        // else{
        //     swerve.drive(new Translation2d(0,0), 0, false, true);
        // }
        // shooter.spinManually(-0.1);
    }
    @Override
    public boolean isFinished() {
        // if(isAuto && AutoConstants.currentCenterNotePos > AutoConstants.howManyNotesAreWeAttempting)
        // {
            
        //     return true;
        // }
        // 
        if(intake.getNoteInIntake() || (timeWithoutTarget > stopIntakeDelay && isAuto)) {
            timeWithoutTarget = 0;
            if(intake.getBackBeam())
            {  
                if(!isAuto) {
                    RumbleManager.rumble(driver, 0.2);
                }
                swerve.drive(new Translation2d(), 0, true, true);
            }
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted){
        arm.stop();
        intake.stop();
        swerve.stopDriving();

    }
}
