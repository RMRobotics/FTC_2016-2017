package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;

/**
 * Created by Peter on 2/1/2017.
 */


public class ZPIDController implements IDataArrivalSubscriber {
    private final Object sync_event = new Object();
    private boolean timestamped = true;
    com.kauailabs.navx.ftc.navXPIDController.navXTimestampedDataSource timestamped_src;
    com.kauailabs.navx.ftc.navXPIDController.navXUntimestampedDataSource untimestamped_src;
    AHRS navx_device;
    long last_system_timestamp = 0L;
    long last_sensor_timestamp = 0L;
    long prev_sensor_timestamp = 0L;
    private double error_current = 0.0D;
    private double error_previous = 0.0D;
    private double error_total = 0.0D;
    private double p;
    private double i;
    private double d;
    private double ff;
    private double max_input = 0.0D;
    private double min_input = 0.0D;
    private double max_output = 1.0D;
    private double min_output = -1.0D;
    private com.kauailabs.navx.ftc.navXPIDController.ToleranceType tolerance_type;
    double tolerance_amount;
    private boolean continuous = false;
    private boolean enabled = false;
    private double setpoint = 0.0D;
    private double result = 0.0D;

    public volatile boolean moving = false;

    public double angular_velocity = 0.0D;
    public double prev_process_value = 0;

    public void untimestampedDataReceived(long curr_system_timestamp, Object kind) {

    }

    public void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object kind) {
        if(this.enabled && kind.getClass() == AHRS.DeviceDataType.class) {
            double process_value;
            switch(this.timestamped_src.ordinal()) {
                case 0:
                    process_value = (double)this.navx_device.getYaw();
                    break;
                default:
                    process_value = 0.0D;
            }

            last_system_timestamp = curr_system_timestamp;
            prev_sensor_timestamp = last_sensor_timestamp;
            last_sensor_timestamp = curr_sensor_timestamp;
            stepController(process_value, 0);
            synchronized(this.sync_event) {
                this.sync_event.notify();
            }
        }
    }

    public void yawReset() {
        if(this.timestamped && this.timestamped_src == com.kauailabs.navx.ftc.navXPIDController.navXTimestampedDataSource.YAW) {
            this.last_sensor_timestamp = 0L;
            this.error_current = 0.0D;
            this.error_previous = 0.0D;
            this.error_total = 0.0D;
            this.result = 0.0D;
        }

    }

    public ZPIDController(AHRS navx_device, com.kauailabs.navx.ftc.navXPIDController.navXTimestampedDataSource src) {
        this.navx_device = navx_device;
        this.timestamped = true;
        this.timestamped_src = src;
        this.setInputRange(-180.0D, 180.0D);
        navx_device.registerCallback(this);
    }

    public ZPIDController(AHRS navx_device, com.kauailabs.navx.ftc.navXPIDController.navXUntimestampedDataSource src) {
        this.navx_device = navx_device;
        this.timestamped = false;
        this.untimestamped_src = src;
        navx_device.registerCallback(this);
    }

    public void close() {
        this.enable(false);
        this.navx_device.deregisterCallback(this);
    }

    public boolean isNewUpdateAvailable(ZPIDController.PIDResult result) {
        boolean new_data_available;
        if(this.enabled && result.timestamp < this.last_sensor_timestamp) {
            new_data_available = true;
            result.on_target = this.isOnTarget();
            result.output = this.get();
            result.timestamp = this.last_sensor_timestamp;
            result.angular_velocity = this.angular_velocity;
            result.error = this.error_current;
        } else {
            new_data_available = false;
        }

        return new_data_available;
    }

    public boolean waitForNewUpdate(ZPIDController.PIDResult result, int timeout_ms) throws InterruptedException {
//        BetterDarudeAutoNav.ADBLog("Wait 1");
        boolean ready = this.isNewUpdateAvailable(result);
//        BetterDarudeAutoNav.ADBLog("Wait 2");
        if(!ready && !Thread.currentThread().isInterrupted()) {
            Object var4 = this.sync_event;
            synchronized(this.sync_event) {
                this.sync_event.wait((long)timeout_ms);
            }

            ready = this.isNewUpdateAvailable(result);
        }
//        BetterDarudeAutoNav.ADBLog("Wait 3");

        return ready;
    }

    public double getError() {
        return this.error_current;
    }

    public synchronized void setTolerance(com.kauailabs.navx.ftc.navXPIDController.ToleranceType tolerance_type, double tolerance_amount) {
        this.tolerance_amount = tolerance_amount;
        this.tolerance_type = tolerance_type;
    }

    public boolean isOnTarget() {
        boolean on_target = false;
        on_target = Math.abs(this.getError()) < this.tolerance_amount;

        return on_target;
    }

    public double stepController(double process_variable, int num_missed_samples) {
        synchronized(this) {
            double adjP = this.p;
            double absErr = Math.abs(error_current);

            this.error_current = this.setpoint - process_variable;

            angular_velocity = process_variable - prev_process_value;
            angular_velocity /= last_sensor_timestamp - prev_sensor_timestamp;
            double dump = 1;

            if(angular_velocity == 0.0 || Math.signum(angular_velocity) == Math.signum(error_current)) {
                // Correcting
                if(!moving) {
                    if (Math.abs(angular_velocity) > 0.11 && absErr < 30) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.09 && absErr < 18) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.075 && absErr < 15.5) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.06 && absErr < 13) {
                        dump = 0; // brake
                    } else if (absErr < 5 && Math.abs(angular_velocity) > 0.01) {
                        dump = 0;  // stop just before target
                    }
                } else {
                    if (absErr < 20 && Math.abs(angular_velocity) > 0.06) {
                        dump = -10;  // stop rotation
                    } else if (Math.abs(angular_velocity) > 0.09 && absErr < 25) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.075 && absErr < 21) {
                        dump = 0; // brake
                    } else if (Math.abs(angular_velocity) > 0.06 && absErr < 17) {
                        dump = 0; // brake
                    }
                }

//                else if(absErr < 20) adjP /=1.7;
            } else {
                // Moving in the wrong direction, apply maz correction
                if(Math.abs(angular_velocity) > 0.02) dump = 10;
            }
//            BetterDarudeAutoNav.ADBLog("av: " + angular_velocity + ", err: " + absErr + ", dump: " + dump + ", mov: " + moving);

            double estimated_i;
            if(this.continuous) {
                estimated_i = this.max_input - this.min_input;
                if(Math.abs(this.error_current) > estimated_i / 2.0D) {
                    if(this.error_current > 0.0D) {
                        this.error_current -= estimated_i;
                    } else {
                        this.error_current += estimated_i;
                    }
                }
            }

            result = adjP * this.error_current * dump;

            this.error_previous = this.error_current;

            if(this.result > this.max_output) {
                this.result = this.max_output;
            } else if(this.result < this.min_output) {
                this.result = this.min_output;
            }

            prev_process_value = process_variable;

            return this.result;
        }
    }

    public synchronized void setP(double p) {
        this.p = p;
//        this.stepController(this.error_previous, 0);
    }

    public synchronized void setContinuous(boolean continuous) {
        this.continuous = continuous;
//        this.stepController(this.error_previous, 0);
    }

    public synchronized double get() {
        return this.result;
    }

    public synchronized void setOutputRange(double min_output, double max_output) {
        if(min_output <= max_output) {
            this.min_output = min_output;
            this.max_output = max_output;
        }

//        this.stepController(this.error_previous, 0);
    }

    public synchronized void setInputRange(double min_input, double max_input) {
        if(min_input <= max_input) {
            this.min_input = min_input;
            this.max_input = max_input;
            this.setSetpoint(this.setpoint);
        }
    }

    public synchronized void setSetpoint(double setpoint) {
        if(this.max_input > this.min_input) {
            if(setpoint > this.max_input) {
                this.setpoint = this.max_input;
            } else if(setpoint < this.min_input) {
                this.setpoint = this.min_input;
            } else {
                this.setpoint = setpoint;
            }
        } else {
            this.setpoint = setpoint;
        }

//        this.stepController(this.error_previous, 0);
    }

    public synchronized double getSetpoint() {
        return this.setpoint;
    }

    public synchronized void enable(boolean enabled) {
        this.enabled = enabled;
        if(!enabled) {
            this.reset();
        }

    }

    public synchronized boolean isEnabled() {
        return this.enabled;
    }

    public synchronized void reset() {
        this.enabled = false;
        this.error_current = 0.0D;
        this.error_previous = 0.0D;
        this.error_total = 0.0D;
        this.result = 0.0D;
    }

    public static enum ToleranceType {
        NONE,
        PERCENT,
        ABSOLUTE;

        private ToleranceType() {
        }
    }

    public static enum navXUntimestampedDataSource {
        RAW_GYRO_X,
        RAW_GYRO_Y,
        RAW_GYRO_Z,
        RAW_ACCEL_X,
        RAW_ACCEL_Y,
        RAW_ACCEL_Z,
        RAW_MAG_X,
        RAW_MAG_Y,
        RAW_MAG_Z;

        private navXUntimestampedDataSource() {
        }
    }

    public static enum navXTimestampedDataSource {
        YAW,
        PITCH,
        ROLL,
        COMPASS_HEADING,
        FUSED_HEADING,
        ALTITUDE,
        LINEAR_ACCEL_X,
        LINEAR_ACCEL_Y,
        LINEAR_ACCEL_Z;

        private navXTimestampedDataSource() {
        }
    }

    public static class PIDResult {
        public double output = 0.0D;
        public long timestamp = 0L;
        public boolean on_target = false;
        public double angular_velocity = 0;
        public double error = 0;

        public PIDResult() {
        }

        public long getTimestamp() {
            return this.timestamp;
        }

        public boolean isOnTarget() {
            return this.on_target;
        }

        public double getStationaryOutput(double min_stationary_output) {
                if (Math.abs(this.output) < min_stationary_output)
                    return Math.signum(this.output) * min_stationary_output;
                else return this.output;
        }
        public double getOutput() {
            return this.output;
        }
    }

    public static enum TimestampType {
        SENSOR,
        SYSTEM;

        private TimestampType() {
        }
    }
}
