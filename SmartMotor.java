package frc.robot.KoiUtils;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * A Motor controller that utilizes an absolute encoder to reset the position of a relative encoder
 * Implements basic PID values
 */
public class SmartMotor {
    public enum MOTOR_STATE {
        COMP,
        DEBUG,
        TUNING
    }

    private final SparkMax motor;
    private final DutyCycleEncoder absoluteEncoder;
    private SparkMaxConfig config = null;
    private SparkClosedLoopController pid = null;
    private String motorName = "Noname";
    private MOTOR_STATE motorState = MOTOR_STATE.COMP;

    // PID cache variables for SmartDashboard tuning
    private double kP, kI, kD, kFF;

    /*
     * A SmartMotor controller constructor for motors that don't require PID
     */
    public SmartMotor(int m_port, MotorType m_type, int a_port) {
        motor = new SparkMax(m_port, m_type);
        absoluteEncoder = new DutyCycleEncoder(a_port, 360, 0);
    }

    /*
     * A SmartMotor controller constructor for motors that don't require PID, with offset implementation
     */
    public SmartMotor(int m_port, MotorType m_type, int a_port, int a_offset) {
        motor = new SparkMax(m_port, m_type);
        absoluteEncoder = new DutyCycleEncoder(a_port, 360, a_offset);
    }

    /*
     * A SmartMotor controller constructor with PID
     */
    public SmartMotor(int m_port, MotorType m_type, int a_port, SparkMaxConfig config) {
        motor = new SparkMax(m_port, m_type);
        absoluteEncoder = new DutyCycleEncoder(a_port, 360, 0);
        this.config = config;
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid = motor.getClosedLoopController();
        cachePIDFromConfig(config);
    }

    /*
     * A SmartMotor controller constructor with PID, with offset implementation
     */
    public SmartMotor(int m_port, MotorType m_type, int a_port, int a_offset, SparkMaxConfig config) {
        motor = new SparkMax(m_port, m_type);
        absoluteEncoder = new DutyCycleEncoder(a_port, 360, a_offset);
        this.config = config;
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid = motor.getClosedLoopController();
        cachePIDFromConfig(config);
    }

    // Cache PID values from the config for dashboard tuning
    private void cachePIDFromConfig(SparkMaxConfig cfg) {
        // Initialize with default values since we can't easily extract from config
        // These will be updated from SmartDashboard in tuning mode
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        kFF = 0.0;
    }

    public SmartMotor setMotorName(String m_name) {
        this.motorName = m_name;
    }

    public SmartMotor setMotorState(MOTOR_STATE state) {
        this.motorState = state;
    }

    // Returns the Smart motor encoder
    public RelativeEncoder getEncoder() {
        return motor.getEncoder();
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    // Syncs the relative encoder to the absolute encoder position
    public void syncToAbsolute() {
        motor.getEncoder().setPosition(absoluteEncoder.get());
    }

    // Motor periodic function, should be put in the subsystem periodic method,
    // responsible for debugging and periodic checks
    public void m_periodic() {
        if (motorState == MOTOR_STATE.COMP)
            return;

        SmartDashboard.putNumber(motorName + " Position", motor.getEncoder().getPosition());

        if (motorState == MOTOR_STATE.DEBUG)
            return;

        // TUNING mode: expose PID values and update config if changed
        SmartDashboard.putNumber(motorName + " kP", kP);
        SmartDashboard.putNumber(motorName + " kI", kI);
        SmartDashboard.putNumber(motorName + " kD", kD);
        SmartDashboard.putNumber(motorName + " kFF", kFF);

        double newP = SmartDashboard.getNumber(motorName + " kP", kP);
        double newI = SmartDashboard.getNumber(motorName + " kI", kI);
        double newD = SmartDashboard.getNumber(motorName + " kD", kD);
        double newFF = SmartDashboard.getNumber(motorName + " kFF", kFF);

        if (Math.abs(newP - kP) > 1e-6 ||
                Math.abs(newI - kI) > 1e-6 ||
                Math.abs(newD - kD) > 1e-6 ||
                Math.abs(newFF - kFF) > 1e-6) {

            kP = newP;
            kI = newI;
            kD = newD;
            kFF = newFF;

            // Update PID constants in config
            config.closedLoop.pid(kP, kI, kD).velocityFF(kFF);

            // Apply updated config to the motor
            motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    /*
     * Move motor to position based on positionConversionFactor units
     * Works with SmartMotion
     */
    public void moveToPosition(double position) {
        if (pid != null) {
            pid.setReference(position, ControlType.kPosition);
        } else {
            KoiLog.logError(motorName + " - A pid function was called for a motor without pid configured");
        }
    }
}