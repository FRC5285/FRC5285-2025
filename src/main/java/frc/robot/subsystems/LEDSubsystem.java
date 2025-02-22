package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Spark ledDriver = new Spark(LEDConstants.pwmChannel);

    public LEDSubsystem() {}

    public Command toNormal() {
        return run(() -> {
            this.ledDriver.set(LEDConstants.normalColor);
        });
    }

    public Command toAuton() {
        return run(() -> {
            this.ledDriver.set(LEDConstants.autonColor);
        });
    }
}
