package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private CANSparkMax elevMotor;
  private RelativeEncoder enc;
  private double maxSpeed; // max manual speed

  private PIDController pid;
  private double previousError;
  private double currentError;
  private boolean pidOn;
  private double topEncLimit;
  private double bottomEncLimit;

  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  private double setpoint;

  public ElevatorSubsystem() {
    elevMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_PORT, MotorType.kBrushless);
    topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LS_PORT);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LS_PORT);
    enc = elevMotor.getEncoder();
    maxSpeed = ElevatorConstants.SPEED_CAP;
    elevMotor.setIdleMode(IdleMode.kBrake);
    topEncLimit = ElevatorConstants.TOP_ENC_LIMIT;
    bottomEncLimit = ElevatorConstants.BOTTOM_ENC_LIMIT;
    pid = new PIDController(0.0, 0.0, 0.0);
    previousError = 0;
    pid.setTolerance(1);
    setpoint = enc.getPosition();

  }

  ////////////////////////
  //  Accessor Methods  //
  ////////////////////////

  public double getEnc() {
    return enc.getPosition();
  }

  public boolean topSwitchPressed() { //should return true if top limit switch pressed
    return topLimitSwitch.get();
  }

  public boolean bottomSwitchPressed() { //should return true if bottom limit switch pressed
    return bottomLimitSwitch.get();
  }

  //////////////////////////////
  //  Basic Movement Methods  //
  //////////////////////////////
  
  // Deadzone includes a speedcap at 0.5 in either direction
  public double deadzone(double speed) {
    if (Math.abs(speed) < 0.1) {
      return 0;
    }
    else if (speed > maxSpeed) {
      return maxSpeed;
    }
    else if (speed < maxSpeed) {
      return maxSpeed;
    }
    else {
      return speed;
    }
  }
  
  //Stops the elevator motor
  public void elevStop(){
    elevMotor.set(0);
  }

  //Changes 
  public void setpointTo(double value){
    setpoint = value;
  }

  // Checks if limit switches are pressed to prevent movement in that direction
  public void ManualElevator(double speed) {
    // if (getTopLimitSwitch() && speed < -0.1) {
    //   elevMotor.set(deadzone(speed));
    // }
    // else if (getBottomLimitSwitch() && speed > 0.1){
    //   elevMotor.set(deadzone(speed));
    // }
    // else if (!getTopLimitSwitch() && !getBottomLimitSwitch()){
    //   elevMotor.set(deadzone(speed));
    // }
    // else{
    //   elevStop();
    // }

    // This line is in case of no limitswitches and just sets motor to joystick speed
    elevMotor.set(deadzone(speed));
  }

  //////////////////
  //  PID Methods //
  //////////////////

  public void enablePid(){
    pidOn = true;
  }

  public void disablePid(){
    pidOn = false;
  }

  public void resetI(){
    currentError = pid.getPositionError();
    
    if (currentError > 0 && previousError < 0){
      pid.reset();
    }
    else if (currentError < 0 && previousError > 0){
      pid.reset();
    }
    
    previousError = currentError;
  }

  @Override
  public void periodic() {
    resetI();

    double pidSpeed = 0;
    if(pidOn){
      pidSpeed = pid.calculate(enc.getPosition(), setpoint);
    }
    else{
      pidSpeed = maxSpeed;
    }

    if(topSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(bottomSwitchPressed() && pidSpeed < 0){
      pidSpeed = 0;
    }

    elevMotor.set(pidSpeed);

    // SmartDashboard
    SmartDashboard.putNumber("Elevator Encoder", getEnc());
    SmartDashboard.putBoolean("Elevator Top LS", topSwitchPressed());
    SmartDashboard.putBoolean("Elevator Bottom LS", bottomSwitchPressed());
  }
}