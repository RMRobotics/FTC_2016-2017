package org.firstinspires.ftc.rmrobotics.opmodes.SandstoRM;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Gregory on 1/10/2017.
 */

public class AutoFxns {

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor shootL;
    private DcMotor shootR;

    private Servo index;
    private CRServo beaconL;
    private CRServo beaconR;

    byte[] colorLcache;
    byte[] colorRcache;
    I2cDevice colorL;
    I2cDevice colorR;
    I2cDevice lineL;
    I2cDevice lineR;
    I2cDeviceSynch colorLreader;
    I2cDeviceSynch colorRreader;
    I2cDeviceSynch lineLreader;
    I2cDeviceSynch lineRreader;
    private ElapsedTime time = new ElapsedTime();


    public AutoFxns(DcMotor a, DcMotor b, DcMotor c, DcMotor d, DcMotor e, DcMotor f, Servo aa, CRServo bb, CRServo cc, I2cDevice aaa, I2cDevice bbb, I2cDevice ccc, I2cDevice ddd, I2cDeviceSynch aaaa, I2cDeviceSynch bbbb, I2cDeviceSynch cccc, I2cDeviceSynch dddd)
    {
        wheelFL = a;
        wheelFR = b;
        wheelBL = c;
        wheelBR = d;
        shootL = e;
        shootR = f;
        index = aa;
        beaconL = bb;
        beaconR = cc;
        colorL = aaa;
        colorR = bbb;
        lineL = ccc;
        lineR = ddd;
        colorLreader = aaaa;
        colorRreader = bbbb;
        lineLreader = cccc;
        lineRreader = dddd;
    }

    public void move(double duration, double power, double angle, double rotate) {
        angle *= (Math.PI / 180);
        time.reset();
        while (time.seconds() < duration) {
            wheelFL.setPower(power * Math.sin(angle + (Math.PI / 4)) + rotate);
            wheelFR.setPower(power * Math.cos(angle + (Math.PI / 4)) - rotate);
            wheelBL.setPower(power * Math.cos(angle + (Math.PI / 4)) + rotate);
            wheelBR.setPower(power * Math.sin(angle + (Math.PI / 4)) - rotate);
        }
    }

    public void shoot(double rev, double fireTime) {
        shootL.setPower(100);
        shootR.setPower(100);
        time.reset();
        while (time.seconds() < rev) ;
        index.setPosition(0);
        time.reset();
        while (time.seconds() < fireTime) ;
        shootL.setPower(0);
        shootR.setPower(0);
    }

    public String scanColor(String color) {
        byte[] colorLcache = colorLreader.read(0x04, 1);
        byte[] colorRcache = colorRreader.read(0x04, 1);
        if (color.charAt(0) == 'r' || color.charAt(0) == 'R') {
            //if left detects red
            if (colorLcache[0] > colorRcache[0])
                return "Left";
            //if right detects red
            if (colorLcache[0] < colorRcache[0])
                return "Right";
        }
        if (color.charAt(0) == 'b' || color.charAt(0) == 'B') {
            //if left detects blue
            if (colorLcache[0] < colorRcache[0])
                return "Left";
            //if right detects blue
            if (colorLcache[0] > colorRcache[0])
                return "Right";
        }
        return "Undetermined";
    }

    public void beaconEngage(boolean set, String side) {
        if (!side.equals("Undetermined")) {
            if (beaconL != null) {
                if (side.charAt(0) == 'L' && set == true)
                    beaconL.setPower(100);
                else if (side.charAt(0) == 'L' && set == false)
                    beaconL.setPower(0);
            }
            if (beaconR != null) {
                if (side.charAt(0) == 'R' && set == true)
                    beaconR.setPower(100);
                else if (side.charAt(0) == 'R' && set == false)
                    beaconR.setPower(0);
            }
        }
    }

    public void lineCheck(double num, String side) {
        //on red side the right side should see line first, on blue side left side should see line first
        if (side.charAt(0) == 'r' || side.charAt(0) == 'R')
        {
            if (colorRreader.read(0x04, 1)[0] > 15) {
                while (colorLreader.read(0x04, 1)[0] <= 15) {
                    wheelFL.setPower(num);
                    wheelFR.setPower(num);
                    wheelBL.setPower(num);
                    wheelBR.setPower(num);
                }
                time.reset();
                while (colorRreader.read(0x04, 1)[0] <= 15) {
                    wheelFL.setPower(-num);
                    wheelFR.setPower(num);
                    wheelBL.setPower(-num);
                    wheelBR.setPower(num);
                }
                double turntime = time.time();
                time.reset();
                move((int)(turntime/2), 0, 0, num);
            }
        }
        if (side.charAt(0) == 'b' || side.charAt(0) == 'B') {
            if (colorLreader.read(0x04, 1)[0] > 15) {
                while (colorRreader.read(0x04, 1)[0] <= 15) {
                    wheelFL.setPower(num);
                    wheelFR.setPower(num);
                    wheelBL.setPower(num);
                    wheelBR.setPower(num);
                }
                time.reset();
                while (colorLreader.read(0x04, 1)[0] <= 15) {
                    wheelFL.setPower(-num);
                    wheelFR.setPower(num);
                    wheelBL.setPower(-num);
                    wheelBR.setPower(num);
                }
                double turntime = time.time();
                time.reset();
                move((int) (turntime / 2), 0, 0, num);
            }
        }
    }
}
