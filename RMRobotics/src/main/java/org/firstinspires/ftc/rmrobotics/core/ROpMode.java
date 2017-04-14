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

abstract class ROpMode extends OpMode {

    private ArrayList<motor> motors = new ArrayList<>();
    private ArrayList<servo> servos =  new ArrayList<>();
    private ArrayList<crservo> crservos = new ArrayList<>();
    protected ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    protected RControl control;
    protected Robot robot;

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

    @Override
    public void start(){
        runtime.reset();
    }

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

    protected abstract Robot setRobot();

    protected void configureHardware() {
        robot = setRobot();
        motors = robot.motors();
        servos = robot.servos();
        crservos = robot.crservos();
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