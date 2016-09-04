package org.firstinspires.ftc.ftc5421.core;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftc5421.control.RControl;
import org.firstinspires.ftc.ftc5421.hardware.crservo;
import org.firstinspires.ftc.ftc5421.hardware.motor;
import org.firstinspires.ftc.ftc5421.hardware.servo;
import org.firstinspires.ftc.ftc5421.util.JSONLoader;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public abstract class ROpMode extends OpMode {

    protected Map<String, motor> motorMap = new HashMap<String, motor>();
    protected Map<String, servo> servoMap =  new HashMap<String, servo>();
    protected Map<String, crservo> crservoMap = new HashMap<String, crservo>();
    protected RControl control;
    private String configPath;

    @Override
    public void init() {
        configPath = setConfigurationPath();
        configureHardware();

        this.control = new RControl(gamepad1, gamepad2);
        for (motor m : motorMap.values()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(m.getRMode());
        }
        for (servo s : servoMap.values()) {
            s.setInitPos();
        }
    }

    @Override
    public void loop() {
        this.updateInput();
        this.calculate();
        this.updateHardware();
    }

    protected abstract void updateInput();

    protected abstract void calculate();

    protected void updateHardware() {
        for (motor m : motorMap.values()) {
            m.setCurrentPower();
        }
        for (servo s : servoMap.values()) {
            s.setCurPosition();
        }
        for (crservo c : crservoMap.values()) {
            c.setCurrentPower();
        }
    }

    protected abstract String setConfigurationPath();

    protected void configureHardware() {
        JSONLoader jsonLoader = null;
        try {
            jsonLoader = new JSONLoader(configPath, hardwareMap);
        } catch (IOException e) {
            e.printStackTrace();
            DbgLog.error(e.getMessage());
        } catch (ParseException e) {
            e.printStackTrace();
            DbgLog.error(e.getMessage());
        } catch (NullPointerException e){
            telemetry.addData("Null","null JSON configureHardware");
        }
        motorMap = jsonLoader.getMotorMap();
        servoMap = jsonLoader.getServoMap();
    }

}