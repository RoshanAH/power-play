package org.firstinspires.ftc.teamcode.components

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU as RevIMU
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.rad
import org.firstinspires.ftc.teamcode.core.Component
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlinx.coroutines.CoroutineScope
import kotlin.math.abs
import kotlin.math.PI
import kotlin.math.sign
import kotlin.math.floor

class IMU (
    map: HardwareMap,
    name: String,
    private val orientation: RevHubOrientationOnRobot
) : Component{

  val imu = map.get(RevIMU::class.java, name)

  var heading: Angle = 0.0.rad
    private set
    
  private var lastMeasuredVel = 0.0
  private var rotations = 0

  var velocity: Double = 0.0
    private set



  override fun init(scope: CoroutineScope){
    val params = RevIMU.Parameters(orientation)
    imu.initialize(params)
    imu.resetYaw()
  }

  override fun start(scope: CoroutineScope){}

  override fun update(scope: CoroutineScope){
    heading = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS).rad
    val rawVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()
    var difference = rawVel - lastMeasuredVel
    if(abs(difference) > PI){
      rotations -= sign(difference).toInt()
    }
    lastMeasuredVel = rawVel

    
    velocity = rotations * 2 * PI + rawVel
  }
}
