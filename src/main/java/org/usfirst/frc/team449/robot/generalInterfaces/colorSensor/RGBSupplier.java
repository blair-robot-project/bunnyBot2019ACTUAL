package org.usfirst.frc.team449.robot.generalInterfaces.colorSensor;

import edu.wpi.first.wpilibj.I2C;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

public class RGBSupplier implements Supplier<int[]> {

    /**
     * The I2C this class is a wrapper on.
     */
    protected final I2C i2c;

    /**
     * Default constructor.
     *
     * @param port                     The I2C port the device is connected to.
     * @param deviceAddress            The address of the device on the I2C bus.
     */
    public RGBSupplier(@NotNull I2C.Port port, int deviceAddress) {
        this.i2c = new I2C(port, deviceAddress);
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public int[] get() {
        byte[] receivedData = new byte[3];
        i2c.transaction(new byte[0], 0, receivedData, 3);

        int[] rgb = new int[3];
        for (int i = 0; i < receivedData.length; i++) {
            rgb[i] = (0x000000FF) & receivedData[i];
        }

        return rgb;
    }
}
