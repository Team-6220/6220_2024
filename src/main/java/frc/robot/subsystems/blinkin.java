/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.RobotContainer;
import frc.robot.Constants.blinkinConstants;

public class blinkin extends SubsystemBase {

  private static blinkin INSTANCE = null;

  /* Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor. 
   *  -1  corresponds to 1000us
   *  0   corresponds to 1500us
   *  +1  corresponds to 2000us
   */
  private static Spark m_blinkin = null;
  private final TunableNumber ledSet = new TunableNumber("Led_Set", .67);
  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort  The PWM port the Blinkin is connected to.
   */
  public blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solid_gold();
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void rainbow() {
    set(-0.99);
  }

  public void solid_gold() {
    set(0.62);
  }

  public void solid_red() {
    set(0.61);
  }
  public void solid_orange() {
    set(0.65);
  }
  public void solid_green() {
    set(0.73); 
  }
  public void solid_blue() {
    set(0.87);
  }
  public void sky_blue() {
    set(0.81);
  }
  public void solid_purple() {
    set(0.91);
  }
  @Override
  public void periodic() {
    if(ledSet.hasChanged()) {
      //set(ledSet.get());
    }
  }
  /**
     * Accesses the static instance of the ArmSubsystem singleton
     * @return ArmSubsystem Singleton Instance
     */
    public static synchronized blinkin getInstance() {
      if (INSTANCE == null) {
          INSTANCE = new blinkin(blinkinConstants.PWMPort);
      }
      return INSTANCE;
  }
}