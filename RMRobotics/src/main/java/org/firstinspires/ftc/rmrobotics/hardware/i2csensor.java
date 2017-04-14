package org.firstinspires.ftc.rmrobotics.hardware;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.rmrobotics.util.enums.Sensors;

/**
 * Created by Simon on 4/14/2017.
 */

public class i2csensor {
    private I2cDevice parent;
    private I2cDeviceSynchImpl reader;
    private Sensors type;
    private byte address;

    public i2csensor(I2cDevice p, byte a, Sensors s) {
        parent = p;
        address = a;
        type = s;
        reader = new I2cDeviceSynchImpl(parent, I2cAddr.create8bit(a), false);
    }

    public void write8(int a, int b) {
        reader.write8(a, b);
    }

    public void engage() {
        reader.engage();
        if (type == Sensors.COLORON) {
            write8(3, 0);
        } else if (type == Sensors.COLOROFF) {
            write8(3, 1);
        }
    }

    public byte[] read(byte a, int n) {
        return reader.read(a, n);
    }
}
