package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final ProfiledPIDController wristPIDController;
  private final WristState wristState;
  private boolean motorOverride = false;

  public WristSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit(10);
    
    wristMotor = new SparkMax(WristConstants.wristMotorID, MotorType.kBrushless);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderID);
    wristEncoder.setInverted(true);

    wristPIDController = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, new TrapezoidProfile.Constraints(WristConstants.maxV, WristConstants.maxA));
    wristPIDController.enableContinuousInput(0.0, 1.0);
    wristPIDController.setTolerance(0.01);

    wristState = new WristState();

    wristPIDController.setGoal(this.getCurrentPosition());
  }

  private Command goToPosition(DoubleSupplier getTargetPosition){
    return runOnce(()-> {
      this.motorOverride = false;
      wristPIDController.setGoal(getTargetPosition.getAsDouble());
    });
  }

  public Command moveDown() {
    return runOnce(() -> wristPIDController.setGoal(wristPIDController.getSetpoint().position + 0.01));
  }

  public Command moveUp() {
    return runOnce(() -> wristPIDController.setGoal(wristPIDController.getSetpoint().position - 0.01));
  }

  @Override
  public void periodic() {
    double calcAmt = wristPIDController.calculate(this.getCurrentPosition());
    if (motorOverride == false) this.wristMotor.set(calcAmt);
  }

  public Command runSmallAmt() {
    return runOnce(() -> {
      this.motorOverride = true;
      wristMotor.set(0.1);
    });
  }

  public Command stopMotor() {
    return runOnce(() -> {
      this.motorOverride = true;
      wristMotor.stopMotor();
    });
  }

  public boolean isAtSetpoint() {
    return wristPIDController.atSetpoint();
  }
  
  public Command goToIntakePosition(){
    return goToPosition(()-> wristState.getIntakePosition());
  }
  
  public Command goToLowShootPosition() {
    return goToPosition(() -> wristState.getLowShootPosition());
  }

  public Command goToMidShootPosition(){
    return goToPosition(()-> wristState.getMidShootPosition());
  }

  public Command goToHighShootPosition(){
    return goToPosition(()-> wristState.getHighShootPosition());
  }
  
  public Command goAllTheWayUp() {
    return goToPosition(() -> 0.0);
  }

  public double getCurrentPosition(){
    double position = wristEncoder.get() - wristState.getEncoderOffset();
    return position < 0.0 ? position + 1.0 : position;
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

      builder.addDoubleProperty("intakePosition", this::getIntakePosition, null);
      builder.addDoubleProperty("lowShootPosition", this::getLowShootPosition, null);
      builder.addDoubleProperty("midShootPosition", this::getMidShootPosition, null);
      builder.addDoubleProperty("highShootPosition", this::getHighShootPosition, null);
      builder.addDoubleProperty("encoderOffset", this::getEncoderOffset, null);
      builder.addDoubleProperty("wristEncoder", ()-> wristEncoder.get(), null);
      builder.addDoubleProperty("currentPosition", () -> getCurrentPosition(), null);
      builder.addDoubleProperty("targetPosition", ()-> wristPIDController.getGoal().position, null);
      builder.addBooleanProperty("atSetpoint", ()-> wristPIDController.atSetpoint(), null);
      builder.addDoubleProperty("setpoint", () -> wristPIDController.getSetpoint().position, null);
      builder.addDoubleProperty("wristMotor", ()-> wristMotor.get(), null);

      builder.addDoubleProperty("wristP", ()-> wristPIDController.getP(), this::setP);
      builder.addDoubleProperty("wristI", ()-> wristPIDController.getI(), this::setI);
      builder.addDoubleProperty("wristD", ()-> wristPIDController.getD(), this::setD);
    }

    public double getCurrentPosition(){
      double position = wristEncoder.get() - getEncoderOffset();
      return position < 0.0 ? position + 1.0 : position;
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
