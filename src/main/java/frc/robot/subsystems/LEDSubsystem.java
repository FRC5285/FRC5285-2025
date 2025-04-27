package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ElevatorSubsystem.elevatorLastSelectedHeight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private Spark ledDriver = new Spark(LEDConstants.pwmChannel);

    public LEDSubsystem() {}

    public Command toNormal() {
        return runOnce(() -> {
            this.ledDriver.set(LEDConstants.normalColor);
        });
    }

    public Command toAuton() {
        return runOnce(() -> {
            this.ledDriver.set(LEDConstants.autonColor);
        });
    }

    public Command reefBranchColors(Supplier<elevatorLastSelectedHeight> elevatorHeight) {
        return run(() -> {
            double setToColor;
            if (elevatorHeight.get() == elevatorLastSelectedHeight.ONE) {
                setToColor = LEDConstants.level1Color;
            } else if (elevatorHeight.get() == elevatorLastSelectedHeight.TWO) {
                setToColor = LEDConstants.level2Color;
            } else if (elevatorHeight.get() == elevatorLastSelectedHeight.THREE) {
                setToColor = LEDConstants.level3Color;
            } else {
                setToColor = LEDConstants.level4Color;
            }
            this.ledDriver.set(setToColor);
        });
    }
}
