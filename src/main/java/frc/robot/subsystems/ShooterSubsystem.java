package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.ShooterConstants;
public class ShooterSubsystem extends SubsystemBase{

    private static ShooterSubsystem INSTANCE = null;
    private TalonFX shooterMotorA;
    private TalonFX shooterMotorB;

    private final TunableNumber Kp = new TunableNumber("Shooter kP", ShooterConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Shooter kI", ShooterConstants.kI);
    private final TunableNumber Kd = new TunableNumber("Shooter kD", ShooterConstants.kD);
    private final TunableNumber Ks = new TunableNumber("Shooter FF Ks", ShooterConstants.kFFkS);
    private final TunableNumber Kv = new TunableNumber("Shooter FF Kv", ShooterConstants.kFFkV);
    private final TunableNumber Ka = new TunableNumber("Shooter FF Ka", ShooterConstants.kFFkA);
    
    public final TunableNumber shooterTestVelocity = new TunableNumber("Shooter Test Velocity Target", 200);

    private PIDController m_controllerA, m_controllerB;

    private SimpleMotorFeedforward feedforwardA, feedforwardB;

    private ShooterSubsystem() {
        shooterMotorA = new TalonFX(ShooterConstants.shooterMotorAID);
        shooterMotorB = new TalonFX(ShooterConstants.shooterMotorBID);

        shooterMotorA.setInverted(ShooterConstants.motorAInverted);
        shooterMotorB.setInverted(ShooterConstants.motorBInverted);

        m_controllerA = new PIDController(Kp.get(), Ki.get(), Kd.get());
        m_controllerB = new PIDController(Kp.get(), Ki.get(), Kd.get());

        feedforwardA = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());
        feedforwardB = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());

        m_controllerA.setTolerance(200); //TODO: Add constants
        m_controllerB.setTolerance(200);
    }

    public double getVelocity(TalonFX motor){
        return motor.getVelocity().getValueAsDouble() * 60;
    }

    public void spinToVelocity(double velocity){
        
        double motorASpeed = m_controllerA.calculate(getVelocity(shooterMotorA), velocity) + feedforwardA.calculate(velocity);
        double motorBSpeed = m_controllerB.calculate(getVelocity(shooterMotorB), velocity) + feedforwardB.calculate(velocity);
        shooterMotorA.set(motorASpeed);
        shooterMotorB.set(motorBSpeed);
        System.out.println("Motor 1: " + motorASpeed + " Velocity: " + getVelocity(shooterMotorA));
        SmartDashboard.putNumber("Target Velocity", velocity);
    }

    public void spinManually(double output){
        shooterMotorA.set(output);
        shooterMotorB.set(output);
        
    }

    public void stop(){
        shooterMotorA.set(0);
        shooterMotorB.set(0);
    }

    public boolean isAtSetpoint() {
      return (m_controllerA.atSetpoint() && m_controllerB.atSetpoint());
    }
    @Override
    public  void periodic(){
        if(Kp.hasChanged()
            || Ki.hasChanged()
            || Kd.hasChanged()
        ){
            m_controllerA.setPID(Kp.get(),Ki.get(),Kd.get());
            m_controllerB.setPID(Kp.get(),Ki.get(),Kd.get());
        }
        if(Ks.hasChanged()
            || Kv.hasChanged()
            || Ka.hasChanged()
        ){
            feedforwardA = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());
            feedforwardB = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());
        }

        SmartDashboard.putNumber("Flywheel A Velocity", getVelocity(shooterMotorA));
        SmartDashboard.putNumber("Flywheel B Velocity", getVelocity(shooterMotorB));

    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

}
