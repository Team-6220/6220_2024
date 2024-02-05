package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
public class ShooterSubsystem extends SubsystemBase{

    private static ShooterSubsystem INSTANCE = null;
    private TalonFX shooterMotorA;
    private TalonFX shooterMotorB;

    private final TunableNumber Kp = new TunableNumber("Shooter kP", ShooterConstants.kP);
    private final TunableNumber Ki = new TunableNumber("Shooter kI", ShooterConstants.kI);
    private final TunableNumber Kd = new TunableNumber("Shooter kD", ShooterConstants.kD);
    private final TunableNumber Ks = new TunableNumber("Shooter FF Ks", ShooterConstants.kFFkS);
    private final TunableNumber Kv = new TunableNumber("Shooter FF Ks", ShooterConstants.kFFkV);
    private final TunableNumber Ka = new TunableNumber("Shooter FF Ks", ShooterConstants.kFFkA);

    private PIDController m_controller;

    private SimpleMotorFeedforward feedforward;

    private ShooterSubsystem() {
        shooterMotorA = new TalonFX(ShooterConstants.shooterMotorAID);
        shooterMotorB = new TalonFX(ShooterConstants.shooterMotorBID);

        shooterMotorA.setInverted(ShooterConstants.motorAInverted);
        shooterMotorB.setInverted(ShooterConstants.motorBInverted);

        m_controller = new PIDController(Kp.get(), Ki.get(), Kd.get());

        feedforward = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());
    }

    public double getVelocity(){
        return 0;
    }

    public double calculate(double setpoint){
        return m_controller.calculate(getVelocity(), setpoint) + feedforward.calculate(getVelocity(), setpoint, 1);
    }

    public void spinToVelocity(double velocity){
        
    }

    @Override
    public  void periodic(){
        if(Kp.hasChanged()
            || Ki.hasChanged()
            || Kd.hasChanged()
        ){
            m_controller.setPID(Kp.get(),Ki.get(),Kd.get());
        }
        if(Ks.hasChanged()
            || Kv.hasChanged()
            || Ka.hasChanged()
        ){
            feedforward = new SimpleMotorFeedforward(Ks.get(), Kv.get(), Ka.get());
        }
    }

    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ShooterSubsystem();
        }
        return INSTANCE;
    }

}
