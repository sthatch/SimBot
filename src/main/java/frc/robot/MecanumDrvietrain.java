package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.Drivetrain;

public class MecanumDrvietrain {
    private MecanumDrive m_robotDrive;

    private WPI_TalonSRX leftFront;
    private WPI_TalonSRX leftFrontFollower;
    private WPI_TalonSRX leftRear;
    private WPI_TalonSRX leftRearFollower;
    private WPI_TalonSRX rightFront;
    private WPI_TalonSRX rightFrontFollower;
    private WPI_TalonSRX rightRear;
    private WPI_TalonSRX rightRearFollower;

    private double longOutput;
    private double latOutput;
    private double YawOutput;

    private double longOutputTarget;
    private double latOutputTarget;
    private double YawOutputTarget;

    /**
     * The initialize method creates the motor object and assigns port values to those objects through the
     * frc.robot.Constants.MotorConstants class. This Mecanum drive is setup with 4 groups of 2 motors.
     */
    public void Initialize(){
        leftFront = new WPI_TalonSRX(MotorConstants.kLeftFrontChannel);
        leftFrontFollower = new WPI_TalonSRX(MotorConstants.kLeftFrontFollowerChannel);
        leftFrontFollower.follow(leftFront);
        leftFrontFollower.setInverted(InvertType.FollowMaster);
        
        leftRear = new WPI_TalonSRX(MotorConstants.kLeftRearChannel);
        leftRearFollower = new WPI_TalonSRX(MotorConstants.kLeftRearFollowerChannel);
        leftRearFollower.follow(leftRear);
        leftRearFollower.setInverted(InvertType.FollowMaster);
    
        rightFront = new WPI_TalonSRX(MotorConstants.kRightFrontChannel);
        rightFrontFollower = new WPI_TalonSRX(MotorConstants.kRightFrontFollowerChannel);
        rightFrontFollower.follow(rightFront);
        rightFrontFollower.setInverted(InvertType.FollowMaster);
    
        rightRear = new WPI_TalonSRX(MotorConstants.kRightRearChannel);
        rightRearFollower = new WPI_TalonSRX(MotorConstants.kRightRearFollowerChannel);    
        rightRearFollower.follow(rightRear);
        rightRearFollower.setInverted(InvertType.FollowMaster);

        // Invert the right side motors.
        // You may need to change or remove this to match your robot.
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        longOutput = 0.0;
        latOutput = 0.0;
        YawOutput = 0.0;

        longOutputTarget = 0.0;
        latOutputTarget = 0.0;
        YawOutputTarget = 0.0;

        m_robotDrive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
    }

    /**
     * Manage method that controls how the drivetrain functions are excuted.
     * 
     * <p> IMPORTANT: The setVelocityTargets function needs to be called periodically before this Manage function is called.
     */
    public void Manage(){
        longOutput = calculateMotorOutput(longOutput, longOutputTarget);    
        latOutput = calculateMotorOutput(latOutput, latOutputTarget);
        YawOutput = YawOutputTarget;
        
        m_robotDrive.driveCartesian(-longOutput, -latOutput, YawOutput);
    }

    /**
   * Set method for inputing velocity targets for Mecanum drivetrain.
   *
   * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param LongitudinalOutput The robot's % output in the longitudinal direction [-1.0..1.0]. Forward is positive.
   * @param LateralOutput The robot's % output in the lateral direction [-1.0..1.0]. Left is positive.
   * @param YawOutput The robot's rotation (yaw) rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   */
    public void setVelocityTargets(double LongitudinalOutput, double LateralOutput, double YawOutput){
        longOutputTarget = LongitudinalOutput;
        latOutputTarget = LateralOutput;
        YawOutputTarget = YawOutput;
    }

    /**
    * Method to limit the rate of acceleration in the robot motion
    *
    * <p>This function calulates the next velocity in reference to the current velocity, a target velocity and
    * a calibratable maximum acceleration.
    *
    * @param output The current velocity
    * @param target The target velocity
    * @param result The returned next velocity value
    */
    private double calculateMotorOutput(double output, double target){
      double result = target;
      double offset = 0.0;
  
      if(output != target){
        /*
        * Calulate the output step
        *
        * MaxOutput will always be 1
        * (MaxOutput / KtTimeToMaxOutput) * (kDefaultPeriod / 1 RTOS Cycle)
        */
        offset = Robot.kDefaultPeriod / Drivetrain.KtTimeToMaxOutput;
  
        if(output < target){
          //Robot is accelerating
          result = output + offset;
  
          //Prevent the new acceleration from exceeding the targt acceleration
          if(result > target){
            result = target;
          }
        }else{
          //Robot is decelerating
          result = output - offset;
  
          //Prevent the new deceleration from exceeding the targt deceleration
          if(result < target){
            result = target;
          }
        }  
      }
      
      return result;
      }
}
