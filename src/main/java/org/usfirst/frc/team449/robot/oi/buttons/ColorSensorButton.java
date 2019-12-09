package org.usfirst.frc.team449.robot.oi.buttons;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.I2C;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.PracticeFrame;
import org.usfirst.frc.team449.robot.generalInterfaces.loggable.Loggable;
import org.usfirst.frc.team449.robot.jacksonWrappers.MappedButton;
import org.usfirst.frc.team449.robot.other.Clock;

/**
 * A button that gets triggered by a reaching a certain color threshold.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class ColorSensorButton extends MappedButton implements Loggable {

	/**
	 * The I2C this class is a wrapper on.
	 */
	protected final I2C i2c;

	/**
	 * Thresholds for red and blue values from color sensor, over which get()
	 * will return true depending on alliance color
	 */
	private final int redThreshold, blueThreshold;

	/**
	 * How long to wait after pinging the RIOduino before pinging it again, in milliseconds.
	 */
	private final long millisBetweenPings;

	/**
	 * Cached color values, since the RIOduino can't be pinged every robot tick
	 */
	private int[] cachedRGB;

	/**
	 * The time that we last pinged the RIOduino to find the sensor reading
	 */
	private long timeLastPinged;

	private final boolean triggerOnAlliance;

	/**
	 * Default constructor.
	 *
	 * @param port          The I2C port the device is connected to.
	 * @param deviceAddress The address of the device on the I2C bus.
	 * @param redThreshold  The threshold for red, over which the sensed object will be identified as red.
	 * @param blueThreshold The threshold for blue, over which the sensed object will be identified as blue.
	 * @param secondsBetweenDuinoPings How long to wait after pinging the RIOduino before pinging it again, in seconds. Defaults 0.05 seconds.
	 */
	@JsonCreator
	public ColorSensorButton(@JsonProperty(required = true) @NotNull I2C.Port port,
							 @JsonProperty(required = true) int deviceAddress,
							 @JsonProperty(required = true) int redThreshold,
							 @JsonProperty(required = true) int blueThreshold,
							 @Nullable Double secondsBetweenDuinoPings,
							 @JsonProperty(required = true) boolean triggerOnAlliance) {
		this.i2c = new I2C(port, deviceAddress);
		this.redThreshold = redThreshold;
		this.blueThreshold = blueThreshold;
		this.millisBetweenPings = secondsBetweenDuinoPings != null ? (long) (secondsBetweenDuinoPings * 1000) : 50;
		this.timeLastPinged = 0;
		this.cachedRGB = new int[3];
		this.triggerOnAlliance = triggerOnAlliance;
	}

	private int[] readRGB() {
		byte[] receivedData = new byte[3];
		i2c.transaction(new byte[0], 0, receivedData, 3);

		int[] rgb = new int[3];
		for (int i = 0; i < receivedData.length; i++) {
			rgb[i] = (0x000000FF) & receivedData[i];
		}

		System.out.println("Red value: " + rgb[0]);
		return rgb;
	}

	/**
	 * Returns whether or not the trigger is active.
	 * <p>
	 * <p>This method will be called repeatedly a command is linked to the Trigger.
	 *
	 * @return whether or not the trigger condition is active.
	 */
	@Override
	public boolean get() {
		if (Clock.currentTimeMillis() >= timeLastPinged + millisBetweenPings) {
			cachedRGB = readRGB();
			timeLastPinged = Clock.currentTimeMillis();
		}

		if ((PracticeFrame.isOnRed() && triggerOnAlliance) || (!PracticeFrame.isOnRed() && !triggerOnAlliance)) {
			return cachedRGB[0] > redThreshold;
		} else {
			return cachedRGB[2] > blueThreshold;
		}
	}

	@NotNull
	@Override
	public String[] getHeader() {
		return new String[]{
				"red",
				"green",
				"blue"
		};
	}

	@NotNull
	@Override
	public  Object[] getData() {
		return new Object[]{
				cachedRGB[0],
				cachedRGB[1],
				cachedRGB[2]
		};
	}

	@Override
	public String getLogName() {
		return "color_sensor";
	}
}
