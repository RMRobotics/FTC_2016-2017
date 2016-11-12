package org.firstinspires.ftc.rmrobotics.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.control.RControl;
import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;

import java.util.ArrayList;

@SuppressWarnings("unchecked")
public abstract class ROpMode extends OpMode {

    private ArrayList<motor> motors = new ArrayList<motor>();
    private ArrayList<servo> servos =  new ArrayList<servo>();
    private ArrayList<crservo> crservos = new ArrayList<crservo>();
    protected ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    protected RControl control;
    protected Robot config;

    @Override
    public void init() {
        configureHardware();

        this.control = new RControl(gamepad1, gamepad2);

        for (motor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(m.getRMode());
        }
        for (servo s : servos) {
            s.setInitPos();
        }
    }

    public void init_loop() {
        addTelemetry();
    }

    @Override
    public void loop() {
        this.updateInput();
        this.calculate();
        this.updateHardware();
        addTelemetry();
    }

    public void stop() {
        for (motor m : motors) {
            m.setPower(0);
        }
        for (crservo c : crservos) {
            c.setPower(0);
        }
        updateHardware();
        addTelemetry();
    }

    protected abstract void updateInput();

    protected abstract void calculate();

    protected void updateHardware() {
        for (motor m : motors) {
            m.setCurrentPower();
        }
        for (servo s : servos) {
            s.setCurPosition();
        }
        for (crservo c : crservos) {
            c.setCurrentPower();
        }
    }

    protected abstract Robot setConfiguration();

    protected void configureHardware() {
        config = setConfiguration();
        motors = config.motors();
        servos = config.servos();
        crservos = config.crservos();
    }

    protected void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        for (motor m : motors) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(m.getParent())), m.getPower());
        }
        for (servo s : servos) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(s.getParent())), s.getPosition());
        }
        for (crservo c : crservos) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(c.getParent())), c.getPower());
        }
    }
}