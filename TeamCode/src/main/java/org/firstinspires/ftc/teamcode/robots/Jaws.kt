package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ReadWriteFile
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.BNO055IMUImpl
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.roshanah.jerky.math.*
import com.roshanah.jerky.utils.*
import com.roshanah.jerky.profiling.*
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Slides
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.components.IMU
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.readFile

class Jaws : Robot() {

  lateinit var drivetrain: Mecanum
  lateinit var slides: Slides
  lateinit var camera: Webcam
  lateinit var imu: IMU


  var active: () -> Boolean = { true }

  private lateinit var voltageSensor: HardwareMap.DeviceMapping<VoltageSensor>

  val low = 0.45
  val medium = 0.7
  val high = 1.0
  val coneHeight = 0.168 / 5

  val alliance: String
  val side: String

  val constants = DriveConstants(
    80.0, 
    100.0, 
    9.528 * 0.5, 
    PSVAConstants(0.0, 0.05, 0.01, 0.0015),
    PSVAConstants(0.0, 0.1, 0.013, 0.002),
    PSVAConstants(0.0, 0.0, 0.015, 0.005),
    DriveValues(1.0, 1.0, 1.1, 1.1)
  )

  init {
    alliance = readFile("alliance.txt")
    side = readFile("side.txt")
  }

  val voltage: Double
    get() = voltageSensor.elementAt(0).getVoltage()

  override fun mapHardware(map: HardwareMap) {
    voltageSensor = map.voltageSensor

    camera = Webcam(map, "camera", "mount", { slides.position }, { drivetrain.relativeVel.y }).apply { kp = 0.05 }

    imu = IMU(map, "imu", jawsHubOrientation)

    drivetrain =
        Mecanum(map, "fl", "fr", "bl", "br").apply {
          trackRadius = constants.trackRadius 
          ticksPerInch = 30.9861111
        }

    slides =
        Slides(map, "left", "right", "claw").apply {
          maxticks = 1125
          p = 10.0
          d = 0.02
          gravity = 0.1

          open = 0.5
          close = 0.8
        }

    addComponents(slides, drivetrain, camera, imu)
  }

  suspend fun place(tolerance: Double = 0.1, condition: () -> Boolean = active) {
    if (!camera.cameraRunning)
        throw IllegalStateException("Cannot run place script, camera is not running")

    var lastClosest = camera.closest ?: return

    val controller = PSVAController(constants, drivetrain.relativeVel)
    val dist = 5.5

    var lastTime = System.nanoTime() * 1e-9

    while (condition()) {

      val time = System.nanoTime() * 1e-9
      val deltaTime = time - lastTime
      lastTime = time

      val closest = camera.closest ?: lastClosest
      lastClosest = closest

      val x = closest.xy
      val v = drivetrain.relativeVel

      val dTheta = abs(closest.theta.rad)
      val alphaScale = 0.25

      val lookAngle = 15.0.deg.rad
      
      val targetVel =
        if (x.magnitude >= dist)
          (closest.theta + 90.0.deg).dir *
          sqrt(2 * constants.maxAcceleration * abs(x.magnitude - dist)) *
          sign(x.magnitude - dist) * (1.0 - dTheta / lookAngle).coerceAtLeast(0.0)
        else Vec2.zero

      val dw =
        if (dTheta * 2 * constants.trackRadius >= tolerance)
          sqrt((constants.maxAcceleration * alphaScale) * dTheta / constants.trackRadius) *
          sign(-closest.theta.rad) -
          (v.x * x.y + v.y * x.x) / x.magnitude * deltaTime
        else 0.0
      val tv = constants.constrain(Pose(targetVel, dw))
      // val tv = constants.constrain(Pose(Vec2.zero, dw))

      controller.update(tv)


      drivetrain.move(constants.psva(constants.wheelsRelative(controller.motion), drivetrain.relativeVel))

      // drivetrain.move(constants.sva(controller.wheels * 1.0))
      // println("a: ${controller.vel.heading}, power: ${constants.psva(controller.wheels, drivetrain.vel).fl}")

      if (tv == Pose.zero) break

      yield()
    }

    drivetrain.move(DriveValues.zero)
    slides.open()
  }

  suspend fun coneStack(cone: Int, dir: Angle, tolerance: Double = 0.5){
    val distToWall = 20.5
    val thresholdDist = 12.0

    val controller = PSVAController(constants, drivetrain.relativeVel)

    while(camera.closest == null && active()){
      val dTheta = abs((imu.heading - dir).rad)
      val tw = if (dTheta * 2 * constants.trackRadius >= tolerance)
          sqrt(constants.maxAcceleration * dTheta / constants.trackRadius) *
          sign((dir - imu.heading).rad)
        else 0.0

      controller.update(Pose(Vec2.zero, tw))
      drivetrain.move(constants.psva(constants.wheelsRelative(controller.motion), drivetrain.relativeVel))
    }

    var lastClosest = camera.closest ?: return

    val lookAngle = 15.0.deg.rad


    while(active()){
      val closest = camera.closest ?: lastClosest
      lastClosest = closest

      val dTheta = abs((imu.heading - dir).rad)

      val targetVel = Vec2(sqrt(2 * constants.maxAcceleration * abs(closest.xy.x)) * sign(closest.xy.x), sqrt(2 * constants.maxAcceleration * distToWall))
      val tw = if (dTheta * 2 * constants.trackRadius >= tolerance)
          sqrt(constants.maxAcceleration * dTheta / constants.trackRadius) *
          sign((dir - imu.heading).rad)
        else 0.0

      val tv = Pose(targetVel * (1.0 - dTheta / lookAngle).coerceAtLeast(0.0), tw)

      controller.update(tv)
      drivetrain.move(constants.psva(constants.wheelsRelative(controller.motion), drivetrain.relativeVel))

      if (closest.xy.y < thresholdDist) break

      yield()
    }

    slides.targetPosition = cone * coneHeight + 0.01
    slides.open()

    follow(vi=drivetrain.relativeVel){
      val stop = interpolate(vf, Pose.zero)
      displace(distToWall - stop.displacement.pos.magnitude)
      append(stop)
    }

    slides.close()
    delay(250L)
    slides.targetPosition = low


    drivetrain.move(DriveValues.zero)
  }

  suspend fun follow(
      theta: Angle = 0.0.rad,
      vi: Pose = Pose.zero,
      builder: ProfileBuilder.() -> Unit,
  ): CompoundProfile{
    val profile = buildProfile(constants, theta, vi, builder)
    val startTime = System.nanoTime() * 1e-9

    while (active()) {
      val time = System.nanoTime() * 1e-9 - startTime
      drivetrain.move(constants.psva(profile(time), drivetrain.relativeVel))
      if (time > profile.length) break
      yield()
    }

    return profile
  }

}

val jawsHubOrientation = RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)


