/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LimeLight;
import frc.lib.drivers.LimeLightControlModes.LedMode;
import frc.lib.util.Debugger;
import frc.robot.Robot;
import frc.robot.commands.DefaultLL;

public class VisionLL extends SubsystemBase {

  public final LimeLight limelight;
  private boolean LEDState;
  private final double goalHeight = 2.5; //meters
  private final double robotHeight = 0.7747; //meters
  private final double robotAngle = 0.872665; //radians
  private double zoneGreen = 0;
  private double zoneYellow = 0; 
  private double zoneBlue = 0; 
  private double zoneRed = 0;

  public double turnKp = 0.021; //0.021
  public double turnKi = 0.0; //0.0
  public double turnKd = 0.0; //0.15

  public int enteredZone = -99;

  /**
   * Creates a new VisionLL.
   */
  public VisionLL() {
    limelight = new LimeLight();

    if(!SmartDashboard.containsKey("Green Zone")) SmartDashboard.putNumber("Green Zone", 0);
    if(!SmartDashboard.containsKey("Yellow Zone")) SmartDashboard.putNumber("Yellow Zone", 0);
    if(!SmartDashboard.containsKey("Blue Zone")) SmartDashboard.putNumber("Blue Zone", 0);
    if(!SmartDashboard.containsKey("Red Zone")) SmartDashboard.putNumber("Red Zone", 0);
    
    //setDefaultCommand(new DefaultLL(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //If disabled and LED-Toggle is false, than leave lights off, else they should be on
    //if(!SmartDashboard.getBoolean("Limelight-LED Toggle", false) && !(RobotContainer.driverController.aButton.get() && (Robot.s_robot_state == RobotState.TELEOP))){
    /*if(Robot.s_robot_state == RobotState.DISABLED && !SmartDashboard.getBoolean("Limelight-LED Toggle", false) && !DriverStation.getInstance().isFMSAttached()){
      if (LEDState == true) {
        limeLightLEDOff();
        LEDState = false;
      }
    } else {
      if (LEDState == false) {
        limeLightLEDOn();
        LEDState = true;
      }
    } */
    enteredZone = getZone();
  }

  public int getZone() {
    if(getLLTargetArea() > zoneGreen) {
      SmartDashboard.putString("Entered Zone", "Green");
      return 0;
    } else if (getLLTargetArea() > zoneYellow) {
      SmartDashboard.putString("Entered Zone", "Yellow");
      return 1;
    } else if (getLLTargetArea() > zoneBlue) {
      SmartDashboard.putString("Entered Zone", "Blue");
      return 2;
    } else if (getLLTargetArea() > zoneRed) {
      SmartDashboard.putString("Entered Zone", "Red");
      return 3;
    } else {
      SmartDashboard.putString("Entered Zone", "Unknown");
      return -99;
    }
  }

  public void limeLightLEDOff(){
    limelight.setLEDMode(LedMode.kforceOff);
  }

  public void limeLightLEDOn(){
    limelight.setLEDMode(LedMode.kforceOn);
  }

  public void setLimeLightLED(boolean b){
    if (b){
        limeLightLEDOn();
        LEDState = true;
    } else{
        limeLightLEDOff();
        LEDState = false;
    }
  }

  public boolean getLEDState() {
    return LEDState;
  }

  public double getLLDegToTarget(){
    return limelight.getdegRotationToTarget();
}

public boolean getLLIsTargetFound(){
    return limelight.getIsTargetFound();
}

public double getLLTargetArea(){
    return limelight.getTargetArea();
}


//d=(h2-h1)/tan(a1+a2)
public double getLLRobotToTargetDistance() {
  double ty = limelight.getdegVerticalToTarget();

  double distance = (goalHeight - robotHeight) / 
                    Math.tan(Math.toRadians(Units.degreesToRadians(ty) + robotAngle));
  return distance;
  }

  public void setLimeLightPipeline(int i) {
    setLimeLightPipeline(i);
  }

  public static void printDebug(String msg) {
    Debugger.println(msg, Robot._visionLL, Debugger.debug2);
  }

  public static void printInfo(String msg) {
    Debugger.println(msg, Robot._visionLL, Debugger.info3);
  }

  public static void printWarning(String msg) {
    Debugger.println(msg, Robot._visionLL, Debugger.warning4);
  }

  public void dashboard() {
    zoneGreen = SmartDashboard.getNumber("Green Zone", zoneGreen);
    zoneYellow = SmartDashboard.getNumber("Yellow Zone", zoneYellow);
    zoneBlue = SmartDashboard.getNumber("Blue Zone", zoneBlue);
    zoneRed = SmartDashboard.getNumber("Red Zone", zoneRed);
  }
}
