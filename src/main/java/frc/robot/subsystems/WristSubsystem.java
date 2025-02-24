package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
  
  private final SparkMax wristMotor;
  private final DutyCycleEncoder wristEncoder;
  private final PIDController wristPIDController;
  private final WristState wristState;

  public WristSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit(20);
    
    wristMotor = new SparkMax(WristConstants.wristMotorID, MotorType.kBrushless);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderID);
    wristEncoder.setInverted(true);

    wristPIDController = new PIDController(1.5, 0.3, 0.1);
    wristPIDController.enableContinuousInput(0.0, 1.0);
    wristPIDController.setTolerance(0.01);

    wristState = new WristState();
  }

  private Command goToPosition(DoubleSupplier getTargetPosition){
    return runOnce(()-> {
        wristPIDController.reset();
        wristPIDController.setSetpoint(getTargetPosition.getAsDouble());
      })
      .andThen(run(()->{
        double speed = wristPIDController.calculate(wristState.getCurrentPosition()); //how fast should we run the motor to get to the current position
        wristMotor.set(speed);
      }))
      .until(()-> wristPIDController.atSetpoint())
      .andThen(()-> wristMotor.stopMotor());
  }

  public boolean isAtSetpoint() {
    return wristPIDController.atSetpoint();
  }
  
  public Command goToIntakePosition(){
    return goToPosition(()-> wristState.getIntakePosition());
  }
  
  public Command goToLowShootPosition() {
    return goToPosition(() -> WristConstants.lowShootPosition);
  }

  public Command goToMidShootPosition(){
    return goToPosition(()-> wristState.getMidShootPosition());
  }

  public Command goToHighShootPosition(){
    return goToPosition(()-> wristState.getHighShootPosition());
  }

  public class WristState implements Sendable{ //for smartdashboard

    private double intakePosition = WristConstants.intakePosition;
    private double lowShootPosition = WristConstants.lowShootPosition;
    private double midShootPosition = WristConstants.midShootPosition;
    private double highShootPosition = WristConstants.highShootPosition;
    private double encoderOffset = WristConstants.encoderOffset;
    
    public WristState(){
      SendableRegistry.add(this, "Wrist State");
      SmartDashboard.putData(this);
    }

    @Override 
    public void initSendable(SendableBuilder builder){
      builder.setSmartDashboardType("RobotPreferences");

      builder.addDoubleProperty("intakePosition", this::getIntakePosition, this::setIntakePosition);
      builder.addDoubleProperty("midShootPosition", this::getLowShootPosition, this::setIntakePosition);
      builder.addDoubleProperty("midShootPosition", this::getMidShootPosition, this::setMidShootPosition);
      builder.addDoubleProperty("highShootPosition", this::getHighShootPosition, this::setHighShootPosition);
      builder.addDoubleProperty("encoderOffset", this::getEncoderOffset, this::setEncoderOffset);
      builder.addDoubleProperty("wristEncoder", ()-> wristEncoder.get(), null);
      builder.addDoubleProperty("currentPosition", ()-> getCurrentPosition(), null);
      builder.addDoubleProperty("targetPosition", ()-> wristPIDController.getSetpoint(), null);
      builder.addBooleanProperty("atSetpoint", ()-> wristPIDController.atSetpoint(), null);
      builder.addDoubleProperty("wristMotor", ()-> wristMotor.get(), null);

      builder.addDoubleProperty("wristP", ()-> wristPIDController.getP(), this::setP);
      builder.addDoubleProperty("wristI", ()-> wristPIDController.getI(), this::setI);
      builder.addDoubleProperty("wristD", ()-> wristPIDController.getD(), this::setD);
    }

    public void setP(double kp){
      wristPIDController.setP(kp);
    }
    public void setI(double ki){
      wristPIDController.setI(ki);
    }
    public void setD(double kd){
      wristPIDController.setD(kd);
    }

    public double getCurrentPosition(){
      double position = wristEncoder.get() - getEncoderOffset();
      return position < 0.0 ? position + 1.0 : position;
    }

    public double getIntakePosition() {
      return intakePosition;
    }

    public void setIntakePosition(double intakePosition) {
      this.intakePosition = intakePosition;
    }

    public double getLowShootPosition() {
      return lowShootPosition;
    }

    public void setLowShootPosition(double lowShootPosition) {
      this.lowShootPosition = lowShootPosition;
    }

    public double getMidShootPosition() {
      return midShootPosition;
    }

    public void setMidShootPosition(double midShootPosition) {
      this.midShootPosition = midShootPosition;
    }

    public double getHighShootPosition() {
      return highShootPosition;
    }

    public void setHighShootPosition(double highShootPosition) {
      this.highShootPosition = highShootPosition;
    }

    public double getEncoderOffset() {
      return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
      this.encoderOffset = encoderOffset;
    }
  }
}
