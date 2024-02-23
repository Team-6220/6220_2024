package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
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

    private final TunableNumber KpA = new TunableNumber("Shooter A kP", ShooterConstants.kP);
    private final TunableNumber KiA = new TunableNumber("Shooter A kI", ShooterConstants.kI);
    private final TunableNumber KdA = new TunableNumber("Shooter A kD", ShooterConstants.kD);
    private final TunableNumber KsA = new TunableNumber("Shooter A FF Ks", ShooterConstants.kFFkS);
    private final TunableNumber KvA = new TunableNumber("Shooter A FF Kv", ShooterConstants.kFFkV);
    private final TunableNumber KaA = new TunableNumber("Shooter A FF Ka", ShooterConstants.kFFkA);

    private final TunableNumber KpB = new TunableNumber("Shooter B kP", ShooterConstants.kP);
    private final TunableNumber KiB = new TunableNumber("Shooter B kI", ShooterConstants.kI);
    private final TunableNumber KdB = new TunableNumber("Shooter B kD", ShooterConstants.kD);
    private final TunableNumber KsB = new TunableNumber("Shooter B FF Ks", ShooterConstants.kFFkS);
    private final TunableNumber KvB = new TunableNumber("Shooter B FF Kv", ShooterConstants.kFFkV);
    private final TunableNumber KaB = new TunableNumber("Shooter B FF Ka", ShooterConstants.kFFkA);
    
    public final TunableNumber shooterTestVelocityA = new TunableNumber("Shooter Test A Velocity Target", 3000);
    public final TunableNumber shooterTestVelocityB = new TunableNumber("Shooter Test B Velocity Target", 3000);

    private PIDController m_controllerA, m_controllerB;

    private SimpleMotorFeedforward feedforwardA, feedforwardB;

    private ShooterSubsystem() {
        shooterMotorA = new TalonFX(ShooterConstants.shooterMotorAID);
        shooterMotorB = new TalonFX(ShooterConstants.shooterMotorBID);

        shooterMotorA.setInverted(ShooterConstants.motorAInverted);
        shooterMotorB.setInverted(ShooterConstants.motorBInverted);

        m_controllerA = new PIDController(KpA.get(), KiA.get(), KdA.get());
        m_controllerB = new PIDController(KpB.get(), KiB.get(), KdB.get());

        feedforwardA = new SimpleMotorFeedforward(KsA.get(), KvA.get(), KaA.get());
        feedforwardB = new SimpleMotorFeedforward(KsB.get(), KvB.get(), KaB.get());

        m_controllerA.setTolerance(100); //TODO: Add constants
        m_controllerB.setTolerance(100);
    }

    public double getVelocity(TalonFX motor){
        return motor.getVelocity().getValueAsDouble() * 60;
    }

    public double[] calculate(double velocity){
        double[] outputs = {
            m_controllerA.calculate(getVelocity(shooterMotorA), velocity) + feedforwardA.calculate(velocity),
            m_controllerB.calculate(getVelocity(shooterMotorB), velocity) + feedforwardB.calculate(velocity)
        };
        return outputs;
    }

    public double[] calculate(double[] velocities){
        double[] outputs = {
            m_controllerA.calculate(getVelocity(shooterMotorA), velocities[0]) + feedforwardA.calculate(velocities[0]),
            m_controllerB.calculate(getVelocity(shooterMotorB), velocities[1]) + feedforwardA.calculate(velocities[1])
        };
        return outputs;
    }

    public void spinToVelocity(double velocity){
        double [] motorOutputs = calculate(velocity);
        double motorASpeed = motorOutputs[0];
        double motorBSpeed = motorOutputs[1];
        shooterMotorA.set(motorASpeed);
        shooterMotorB.set(motorBSpeed);
        SmartDashboard.putNumber("Target Velocity A", velocity);
        SmartDashboard.putNumber("Target Velocity B", velocity);
    }

    public void spinToVelocity(double[] velocities){
        double [] motorOutputs = calculate(velocities);
        double motorASpeed = motorOutputs[0];
        double motorBSpeed = motorOutputs[1];
        shooterMotorA.set(motorASpeed);
        shooterMotorB.set(motorBSpeed);
        SmartDashboard.putNumber("Target Velocity A", velocities[0]);
        SmartDashboard.putNumber("Target Velocity B", velocities[1]);
    }
    public void spinToVelocity(Pair<Double, Double> velocities){
        double[] newVelocities = {velocities.getFirst(), velocities.getSecond()};
        spinToVelocity(newVelocities);
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
        if(KpA.hasChanged()
            || KiA.hasChanged()
            || KdA.hasChanged()
        ){
            m_controllerA.setPID(KpA.get(),KiA.get(),KdA.get());
        }
        if(KpB.hasChanged()
            || KiB.hasChanged()
            || KdB.hasChanged()
        ){
            m_controllerB.setPID(KpB.get(),KiB.get(),KdB.get());
        }
        if(KsA.hasChanged()
            || KvA.hasChanged()
            || KaA.hasChanged()
        ){
            feedforwardA = new SimpleMotorFeedforward(KsA.get(), KvA.get(), KaA.get());
        }
        if(KsB.hasChanged()
            || KvB.hasChanged()
            || KaB.hasChanged()
        ){
            feedforwardB = new SimpleMotorFeedforward(KsB.get(), KvB.get(), KaB.get());
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
