package org.firstinspires.ftc.ftc5421.core;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftc5421.control.RControl;
import org.firstinspires.ftc.ftc5421.hardware.RMotor;
import org.firstinspires.ftc.ftc5421.hardware.RServo;
import org.firstinspires.ftc.ftc5421.util.JSONLoader;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public abstract class ROpMode extends OpMode {

    protected Map<String, RMotor> motorMap = new HashMap<String, RMotor>();
    protected Map<String, RServo> servoMap =  new HashMap<String, RServo>();
    protected RControl control;
    private String configPath;

    @Override
    public void init() {
        configPath = setConfigurationPath();
        configureHardware();

        this.control = new RControl(gamepad1, gamepad2);
        for (RMotor m : motorMap.values()) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (RServo s : servoMap.values()) {
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
        for (RMotor m : motorMap.values()) {
            m.updateCurrentPower();
            m.setCurrentPower();
        }
        for (RServo s : servoMap.values()) {
            s.updateCurrentPosition();
            s.setPosition();
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
        telemetry.addData("lolololol", "lolololol"); //Placeholder until further action taken on incorporating or phasing out JSONLoader
    }

}