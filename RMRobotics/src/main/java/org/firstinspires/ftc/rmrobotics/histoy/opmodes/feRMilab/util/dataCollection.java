package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab.util;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.rmrobotics.core.DataLogger;

@TeleOp(name = "feRMi - DataCollection", group = "feRMi")
@Disabled
public class dataCollection extends OpMode{

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private Servo harvester;
    private Servo index;
    private Servo beaconArm;

    private DeviceInterfaceModule dim;
    private I2cDevice range;
    private I2cDeviceSynch rangeReader;
    private I2cDevice colorLeft;
    private I2cDeviceSynch colorLeftReader;
    private DigitalChannel beaconToggle;
    private DigitalChannel dataCollect;

    private DataLogger dataLogger;

    final int BLUE_LED_CHANNEL = 0;
    final int RED_LED_CHANNEL = 1;
    private boolean BLUE_LED = false;
    private boolean RED_LED = true;

    protected AHRS navx;

    @Override
    public void init() {
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        colorLeft = hardwareMap.i2cDevice.get("colorLeft");
        colorLeftReader = new I2cDeviceSynchImpl(colorLeft, I2cAddr.create8bit(0x72), false);
        colorLeftReader.engage();
        colorLeftReader.write8(3,1);

        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceSynchImpl(range, I2cAddr.create8bit(0x60), false);
        rangeReader.engage();

        beaconToggle = hardwareMap.digitalChannel.get("toggle");
        beaconToggle.setMode(DigitalChannelController.Mode.INPUT);
        dataCollect = hardwareMap.digitalChannel.get("data");
        dataCollect.setMode(DigitalChannelController.Mode.INPUT);

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        harvester = hardwareMap.servo.get("h");
        beaconArm = hardwareMap.servo.get("swingArm");
        index = hardwareMap.servo.get("indexer");
        beaconArm.setPosition(0.5);
        beaconArm.setPosition(0.5);

        dim.setLED(RED_LED_CHANNEL, RED_LED);
        dim.setLED(BLUE_LED_CHANNEL, BLUE_LED);

        // navx initialization and calibration
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        while (navx.isCalibrating()) {
            telemetry.addData("Status", !navx.isCalibrating());
            telemetry.update();
        }
        navx.zeroYaw();

        dataLogger = new DataLogger("FirstTestCuz");
    }

    @Override
    public void loop() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        if(beaconToggle.getState()){
            LEDToggle();
            dim.setLED(RED_LED_CHANNEL, RED_LED);
            dim.setLED(BLUE_LED_CHANNEL, BLUE_LED);
        }

        if(dataCollect.getState()){
            dataLogger.newLine();
            dataLogger.addBeacon(colorLeftReader.read(0x04, 1)[0]);
            dataLogger.addSensors(rangeReader.read(0x04, 2)[0]);
//            dataLogger.addEncoders(FL.getCurrentPosition(),FR.getCurrentPosition(),BR.getCurrentPosition(),BL.getCurrentPosition());
//            dataLogger.addTarEncoders(FL.getTargetPosition(),FR.getTargetPosition(),BR.getTargetPosition(),BL.getTargetPosition());
//            dataLogger.addServos(harvester.getPosition(),beaconArm.getPosition(),index.getPosition());
            //Add navx data as well
        }

        telemetry.addData("Color", colorLeftReader.read(0x04, 1)[0] & 0xFF);
        telemetry.addData("Red", colorLeftReader.read(0x05, 1)[0] & 0xFF);
        telemetry.addData("Blue", colorLeftReader.read(0x07, 1)[0] & 0xFF);
        telemetry.addData("Range", rangeReader.read(0x04, 1)[0]); //Ultrasonic in cm
        telemetry.addData("Toggle", beaconToggle.getState());
        telemetry.addData("Data", dataCollect.getState());

    }

    public void stop(){
        dataLogger.closeDataLogger();
    }

    private void LEDToggle(){
        if(RED_LED){
            RED_LED = false;
            BLUE_LED = true;
        }else if(BLUE_LED){
            BLUE_LED = false;
            RED_LED = true;
        }
    }

}
