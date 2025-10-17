package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final CANdle candle = new CANdle(0);

    public LED() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.statusLedOffWhenActive = false;
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config);
    }

    public void setWhiteGRB() {
        candle.setLEDs(255, 255, 255);
    }

    public void setRedGRB() {
        candle.setLEDs(255, 0, 0);
    }

    public void setBlueGRB() {
        candle.setLEDs(0, 0, 255, 0, 0, 300);
    }

    public void setRainbowAnimation() {
        RainbowAnimation rainbow = new RainbowAnimation(1, .3, 300);
        candle.animate(rainbow);
    }
}
