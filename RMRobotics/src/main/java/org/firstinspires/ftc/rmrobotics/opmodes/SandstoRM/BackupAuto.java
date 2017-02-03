package org.firstinspires.ftc.rmrobotics.opmodes.SandstoRM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Gregory on 12/13/2016.
 */
@Autonomous(name="DarudeAutoSensorsR", group="SandstoRM")
public class BackupAuto extends LinearOpMode {

    private DcMotor wheelFL;
    private DcMotor wheelFR;
    private DcMotor wheelBL;
    private DcMotor wheelBR;
    private DcMotor shootL;
    private DcMotor shootR;
    private Servo index;
    private CRServo beaconL;
    private CRServo beaconR;
    I2cDevice colorL;
    I2cDevice colorR;
    I2cDevice lineL;
    I2cDevice lineR;
    I2cDeviceSynch colorLreader;
    I2cDeviceSynch colorRreader;
    I2cDeviceSynch lineLreader;
    I2cDeviceSynch lineRreader;

    public void runOpMode()
    {
        wheelFL = hardwareMap.dcMotor.get("wheelFL");
        wheelFR = hardwareMap.dcMotor.get("wheelFR");
        wheelBL = hardwareMap.dcMotor.get("wheelBL");
        wheelBR = hardwareMap.dcMotor.get("wheelBR");
        shootL = hardwareMap.dcMotor.get("shootL");
        shootR = hardwareMap.dcMotor.get("shootR");
        wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBL.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        beaconL = hardwareMap.crservo.get("beaconL");
        beaconR = hardwareMap.crservo.get("beaconR");
        index = hardwareMap.servo.get("index");

        colorL = hardwareMap.i2cDevice.get("cL");
        colorR = hardwareMap.i2cDevice.get("cR");
        lineL = hardwareMap.i2cDevice.get("colorLineL");
        lineR = hardwareMap.i2cDevice.get("colorLineR");
        colorLreader = new I2cDeviceSynchImpl(colorL, I2cAddr.create8bit(0x50), false);
        colorRreader = new I2cDeviceSynchImpl(colorR, I2cAddr.create8bit(0x52), false);
        lineLreader = new I2cDeviceSynchImpl(lineL, I2cAddr.create8bit(0x3c), false);
        lineRreader = new I2cDeviceSynchImpl(lineR, I2cAddr.create8bit(0x3c), false);
        colorLreader.engage();
        colorRreader.engage();
        lineLreader.engage();
        lineRreader.engage();
        colorLreader.write8(3, 1);
        colorRreader.write8(3, 1);
        lineLreader.write8(3,0);
        lineRreader.write8(3,0);

        //beaconR does not exist on SandstoRM
        AutoFxns auto = new AutoFxns(wheelFL, wheelFR, wheelBL, wheelBR, shootL, shootR, index, beaconL, null, colorL, colorR, lineL, lineR, colorLreader, colorRreader, lineLreader, lineRreader);
        waitForStart();
        //Strategy: move forward, shoot, move forward more to hit beacon, hopefully stop on center platform

        //duration, power, strafe, rotate
        auto.move(3.5,100,0,0);
        //time to rev flywheel, time for ball to exit flywheel
        auto.shoot(3,2.6);
        auto.move(2,100,0,0);
        auto.move(5,0,100,0);
    }
}
