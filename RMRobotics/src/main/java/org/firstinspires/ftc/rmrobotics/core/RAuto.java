package org.firstinspires.ftc.rmrobotics.core;

public abstract class RAuto extends ROpMode {

    @Override
    public void init() {
        super.init();
    }

    public abstract void runState();

    @Override
    public void loop(){
        runState();
        addTelemetry();
    }

}
