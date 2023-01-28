package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import frc.robot.utilities.SplitSlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.Drivetrain.Rate;

/**User tank drive command
 * One joystick for the left side, and one for the right side
 * Applies a nonlinear velocity mapping to the joysticks, and limits the acceleration and deceleration of the drivetrain 
 */
public class UserTankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final SplitSlewRateLimiter m_leftRateLimiter;
  private final SplitSlewRateLimiter m_rightRateLimiter;
  private final DoubleSupplier m_leftInput;
  private final DoubleSupplier m_rightInput;
  private final BooleanSupplier m_boostInput;

  public UserTankDrive(DoubleSupplier leftSupplier, DoubleSupplier rightupplier, BooleanSupplier boostSupplier, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_leftRateLimiter = new SplitSlewRateLimiter(Rate.driverAccel, Rate.driverDeccel);
    m_rightRateLimiter = new SplitSlewRateLimiter(Rate.driverAccel, Rate.driverDeccel);
    m_leftInput = leftSupplier;
    m_rightInput = rightupplier;
    m_boostInput = boostSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  @Override
  public void initialize(){
      m_drivetrain.brakeMode(false);
  }

  @Override
  public void execute(){
    double leftSpeed = MathUtil.clamp(m_leftInput.getAsDouble(), -1, 1);
    double rightSpeed = MathUtil.clamp(m_rightInput.getAsDouble(), -1, 1);
    
    double leftDriveSpeed = leftSpeed * (m_boostInput.getAsBoolean() ? Rate.maxSpeed : Rate.driverSpeed);
    double rightDriveSpeed = rightSpeed * (m_boostInput.getAsBoolean() ? Rate.maxSpeed : Rate.driverSpeed);

    m_drivetrain.driveWheelSpeeds(new DifferentialDriveWheelSpeeds(m_leftRateLimiter.calculate(leftDriveSpeed), m_rightRateLimiter.calculate(rightDriveSpeed)));
  }

  @Override
  public void end(boolean interrupted){
    m_drivetrain.driveWheelSpeeds(new DifferentialDriveWheelSpeeds(0,0));
  }
}