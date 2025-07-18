# SmartMotor Utility Class

A comprehensive motor controller wrapper for FRC robots that combines SparkMax motor controllers with absolute encoders for enhanced position control and live PID tuning capabilities.

## Features

- **Absolute Encoder Integration**: Automatically syncs relative encoder position with absolute encoder for accurate positioning
- **Live PID Tuning**: Real-time PID parameter adjustment through SmartDashboard
- **Multi-State Operation**: Different modes for competition, debugging, and tuning
- **Position Control**: Built-in position-based movement with PID control
- **Flexible Configuration**: Multiple constructor options for different use cases

## Quick Start

### Basic Motor (No PID)
```java
// Motor without PID control
SmartMotor motor = new SmartMotor(1, MotorType.kBrushless, 0);
motor.setMotorName("Arm Motor");
```

### Motor with PID Control
```java
// Create SparkMax configuration with PID values
SparkMaxConfig config = new SparkMaxConfig();
config.closedLoop.pid(0.1, 0.0, 0.01).velocityFF(0.0);

// Create motor with PID
SmartMotor pidMotor = new SmartMotor(2, MotorType.kBrushless, 1, config);
pidMotor.setMotorName("Shooter Angle");
pidMotor.setMotorState(SmartMotor.MOTOR_STATE.TUNING);
```

### With Absolute Encoder Offset
```java
// Motor with 90-degree offset on absolute encoder
SmartMotor offsetMotor = new SmartMotor(3, MotorType.kBrushless, 2, 90, config);
```

## Constructor Options

| Constructor | Parameters | Use Case |
|-------------|------------|----------|
| `SmartMotor(int m_port, MotorType m_type, int a_port)` | Motor port, motor type, encoder port | Basic motor without PID |
| `SmartMotor(int m_port, MotorType m_type, int a_port, int a_offset)` | + encoder offset | Basic motor with encoder offset |
| `SmartMotor(int m_port, MotorType m_type, int a_port, SparkMaxConfig config)` | + PID configuration | Motor with PID control |
| `SmartMotor(int m_port, MotorType m_type, int a_port, int a_offset, SparkMaxConfig config)` | + encoder offset + PID | Full-featured motor |

## Motor States

### COMP (Competition)
- **Purpose**: Production mode for competitions
- **Behavior**: Minimal processing, no dashboard updates
- **Use When**: During actual matches or when performance is critical

### DEBUG  
- **Purpose**: Basic debugging and monitoring
- **Behavior**: Displays motor position on SmartDashboard
- **Use When**: Testing motor movement and encoder readings

### TUNING
- **Purpose**: Live PID parameter adjustment
- **Behavior**: Exposes PID values on SmartDashboard and applies changes in real-time
- **Use When**: Tuning PID parameters during practice or testing

## Usage Example

```java
public class ArmSubsystem extends SubsystemBase {
    private SmartMotor armMotor;
    
    public ArmSubsystem() {
        // Configure motor with PID
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0.05, 0.0, 0.001).velocityFF(0.0);
        
        // Create motor with 180-degree encoder offset
        armMotor = new SmartMotor(5, MotorType.kBrushless, 3, 180, config);
        armMotor.setMotorName("Arm");
        armMotor.setMotorState(SmartMotor.MOTOR_STATE.DEBUG);
    }
    
    @Override
    public void periodic() {
        // IMPORTANT: Must call this in subsystem periodic!
        armMotor.m_periodic();
    }
    
    public void moveToAngle(double angle) {
        armMotor.moveToPosition(angle);
    }
    
    public void syncEncoder() {
        armMotor.syncToAbsolute();
    }
    
    public double getPosition() {
        return armMotor.getPosition();
    }
}
```

## Key Methods

### Setup Methods
- `setMotorName(String name)`: Set the motor name for SmartDashboard identification
- `setMotorState(MOTOR_STATE state)`: Change the motor's operational state

### Control Methods
- `moveToPosition(double position)`: Move motor to specified position using PID
- `syncToAbsolute()`: Sync relative encoder to absolute encoder position

### Information Methods
- `getEncoder()`: Get the underlying RelativeEncoder object
- `getPosition()`: Get current motor position in configured units

### Required Method
- `m_periodic()`: **MUST** be called in your subsystem's `periodic()` method

## SmartDashboard Integration

When in TUNING mode, the following values appear on SmartDashboard:

- `[MotorName] Position`: Current motor position
- `[MotorName] kP`: Proportional gain
- `[MotorName] kI`: Integral gain  
- `[MotorName] kD`: Derivative gain
- `[MotorName] kFF`: Feed-forward gain

Changes to PID values are automatically applied to the motor controller.

## Important Notes

### ⚠️ Critical Requirements
- **Must call `m_periodic()`** in your subsystem's `periodic()` method
- **PID-enabled constructors only** support `moveToPosition()` - calling it on basic motors logs an error
- **Absolute encoder must be properly wired** and configured for sync functionality to work

### 🔧 Technical Details
- Uses `DutyCycleEncoder` with 360-degree range
- PID changes are detected using a threshold of `1e-6` (0.000001)
- Motor configuration persists parameters to SparkMax memory
- Supports REV Robotics SparkMax controllers with 2025 WPILib

### 🎯 Best Practices
- Use descriptive motor names for easy SmartDashboard identification
- Start with DEBUG mode, then switch to TUNING for PID adjustment
- Switch to COMP mode for competitions to maximize performance
- Call `syncToAbsolute()` during robot initialization or when needed

## Troubleshooting

| Problem | Solution |
|---------|----------|
| PID not working | Ensure you used a constructor with `SparkMaxConfig` parameter |
| No SmartDashboard values | Check that `m_periodic()` is called and motor state is DEBUG/TUNING |
| Position jumps unexpectedly | Verify absolute encoder wiring and offset configuration |
| Motor not moving | Check motor state and ensure PID values are reasonable |

## Dependencies

- REV Robotics SparkMax Library (2025)
- WPILib (2025)
- Custom `KoiLog` utility for error logging

## License

This utility is part of the KoiUtils package for FRC Team use.