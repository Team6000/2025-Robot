package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class LEDs extends SubsystemBase {
    private static AddressableLED leds = new AddressableLED(0);
    private static AddressableLEDBuffer buffer = new AddressableLEDBuffer(10);

    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Millimeters.of(16.5);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow =
    m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    public LEDs() {
        leds.setLength(10);
        leds.start();

    }

    @Override
    public void periodic() {
        m_scrollingRainbow.applyTo(buffer);
        leds.setData(buffer);
    }
}
