/**
 * @file java/frc/robot/subsystems/DriveTrain.java
 * @brief Drive train is super cool
 */
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import static frc.robot.Constants.MotorConstants.*;

public class DriveTrain extends SubsystemBase {

    /**
     * Motor variables
     */
    private final CANSparkMax m_frontRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_frontLeftMotor = new CANSparkMax(BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_backLeftMotor = new CANSparkMax(BACK_LEFT_MOTOR_PORT, MotorType.kBrushless);
    private final CANSparkMax m_backRightMotor = new CANSparkMax(FRONT_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    /**
     * Drive variables
     */
    private final DifferentialDrive m_drive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor); // Main robot drive

    // Trigger for full speed
    public final Trigger atFullSpeed = new Trigger(() -> MathUtil.isNear(0.5, m_frontLeftMotor.get(), 0.05));

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain() {
        m_frontLeftMotor.restoreFactoryDefaults();
        m_frontRightMotor.restoreFactoryDefaults();
        m_backLeftMotor.restoreFactoryDefaults();
        m_backRightMotor.restoreFactoryDefaults();

        // Setup inversion
        m_frontRightMotor.setInverted(true);

        // Setup primary and follower controllers
        m_backLeftMotor.follow(m_frontLeftMotor);
        m_backRightMotor.follow(m_frontRightMotor);

        // Motors should brake when no active inputs are given
        m_frontLeftMotor.setIdleMode(IdleMode.kBrake);
        m_backLeftMotor.setIdleMode(IdleMode.kBrake);
        m_frontRightMotor.setIdleMode(IdleMode.kBrake);
        m_backRightMotor.setIdleMode(IdleMode.kBrake);

        // Burn! (i.e. flash the configs to the motors)
        m_frontLeftMotor.burnFlash();
        m_backLeftMotor.burnFlash();
        m_frontRightMotor.burnFlash();
        m_backRightMotor.burnFlash();
    }

    /**
     * Run the drive train in arcade-drive mode.
     *
     * @param forward The forward velocity/movement
     * @param rotation The rotation to use
     */
    public Command runArcadeDrive(DoubleSupplier forward, DoubleSupplier rotation) {
        // Inline construction of command goes here.
        // subsystem::run implicitly requires `this` subsystem.
        return run(
                () -> {
                    m_drive.arcadeDrive(forward.getAsDouble() * Constants.MAX_SPEED, rotation.getAsDouble() * Constants.MAX_SPEED);
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("at full speed", atFullSpeed.getAsBoolean());
        SmartDashboard.putNumber("speed", getSpeed());
    }

    public double getSpeed() {
        return m_frontLeftMotor.get();
    }
}
