package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final int CANdle_ID = 15;
    private final int LED_STRIP_COUNT = 300;
    private final CANdle candle = new CANdle(CANdle_ID);

    public LED() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.statusLedOffWhenActive = false;
        config.brightnessScalar = 1.0;
        candle.configAllSettings(config);
    }

    public void setWhiteGRB() {
        candle.setLEDs(255, 255, 255, 0, 0, LED_STRIP_COUNT);
    }

    public void setRedGRB() {
        candle.setLEDs(255, 0, 0, 0, 0, LED_STRIP_COUNT);
    }

    public void setBlueGRB() {
        candle.setLEDs(0, 0, 255, 0, 0, LED_STRIP_COUNT);
    }

    public void setColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
    }

    public void setRainbowAnimation() {
        RainbowAnimation rainbow = new RainbowAnimation(1, .3, LED_STRIP_COUNT);
        candle.animate(rainbow);
    }

    public void setTwinkleAnimation() {
        // Twinkle Mater Dei Red
        TwinkleAnimation twinkle = new TwinkleAnimation(250, 4, 8);
        candle.animate(twinkle);
    }

    public void setFireAnimation() {
        FireAnimation fire = new FireAnimation(1.0, 0.2, LED_STRIP_COUNT, 1.0, 0.00001);
        candle.animate(fire);
    }
}
