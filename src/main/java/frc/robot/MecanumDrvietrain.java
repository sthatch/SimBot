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

    private double longitudinalVel;
    private double lateralVel;
    private double YawVel;

    private double longitudinalVelTarget;
    private double lateralVelTarget;
    private double YawVelTarget;

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

        longitudinalVel = 0.0;
        lateralVel = 0.0;
        YawVel = 0.0;

        longitudinalVelTarget = 0.0;
        lateralVelTarget = 0.0;
        YawVelTarget = 0.0;

        m_robotDrive = new MecanumDrive(leftFront, leftRear, rightFront, rightRear);
    }

    /**
     * Manage method that controls how the drivetrain functions are excuted.
     * 
     * <p> IMPORTANT: The setVelocityTargets function needs to be called periodically before this Manage function is called.
     */
    public void Manage(){
        longitudinalVel = calculateVelocity(longitudinalVel, longitudinalVelTarget);    
        lateralVel = calculateVelocity(lateralVel, lateralVelTarget);
        YawVel = YawVelTarget;
        
        m_robotDrive.driveCartesian(-longitudinalVel, -lateralVel, YawVel);
    }

    /**
   * Set method for inputing velocity targets for Mecanum drivetrain.
   *
   * <p>Angles are measured counterclockwise from the positive X axis. The robot's speed is
   * independent of its angle or rotation rate.
   *
   * @param LongitudinalVelocity The robot's speed in the longitudinal direction [-1.0..1.0]. Forward is positive.
   * @param LateralVelocity The robot's speed in the lateral direction [-1.0..1.0]. Left is positive.
   * @param YawVelocity The robot's rotation (yaw) rate around the Z axis [-1.0..1.0]. Counterclockwise is
   *     positive.
   */
    public void setVelocityTargets(double LongitudinalVelocity, double LateralVelocity, double YawVelocity){
        longitudinalVelTarget = LongitudinalVelocity;
        lateralVelTarget = LateralVelocity;
        YawVelTarget = YawVelocity;
    }

    /**
    * Method to limit the rate of acceleration in the robot motion
    *
    * <p>This function calulates the next velocity in reference to the current velocity, a target velocity and
    * a calibratable maximum acceleration.
    *
    * @param velocity The current velocity
    * @param target The target velocity
    * @param result The returned next velocity value
    */
    private double calculateVelocity(double velocity, double target){
        double result = target;
    
        if(velocity != target){
          if(velocity < target){
            //Robot is accelerating
            result = velocity + Drivetrain.kMaxAcceleration;
    
            //Prevent the new acceleration from exceeding the targt acceleration
            if(result > target){
              result = target;
            }
          }else{
            //Robot is decelerating
            result = velocity - Drivetrain.kMaxAcceleration;
    
            //Prevent the new deceleration from exceeding the targt deceleration
            if(result < target){
              result = target;
            }
          }  
        }
        
        return result;
      }
}
