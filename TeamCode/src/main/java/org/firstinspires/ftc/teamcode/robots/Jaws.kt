package org.firstinspires.ftc.teamcode.robots

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Slides
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.teamcode.components.PSVAController
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ReadWriteFile
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.PSVAConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.deg
import kotlinx.coroutines.yield
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt
import kotlin.math.pow

class Jaws : Robot() {

  lateinit var drivetrain: Mecanum
  lateinit var slides: Slides
  lateinit var camera: Webcam

  private lateinit var voltageSensor: HardwareMap.DeviceMapping<VoltageSensor>

  val low = 0.45
  val medium = 0.7
  val high = 1.0

  val alliance: String
  val side: String

  val constants = DriveConstants(80.0, 100.0, 9.528 * 0.5, PSVAConstants(0.005, 0.1, 0.01, 0.003))

  init {
    val allianceFile = AppUtil.getInstance().getSettingsFile("alliance.txt")
    val sideFile = AppUtil.getInstance().getSettingsFile("side.txt")

    alliance = ReadWriteFile.readFile(allianceFile) 
    side = ReadWriteFile.readFile(sideFile) 
  }


  val voltage: Double
    get() = voltageSensor.elementAt(0).getVoltage()

  override fun mapHardware(map: HardwareMap) {
    voltageSensor = map.voltageSensor

    camera = Webcam(map, "camera", "mount"){
      drivetrain.relativeVel.y
    }.apply{
      kp = 0.05
    }

    drivetrain = Mecanum(map, "fl", "fr", "bl", "br").apply {
      trackWidth = constants.trackRadius * 2
      ticksPerInch = 30.9861111
      ticksPerDegree = 4.98611
    }

    slides = Slides(map, "left", "right", "claw").apply {
      maxticks = 1125
      p = 10.0
      d = 0.02
      gravity = 0.1

      open = 0.5
      close = 0.8
    }

    addComponents(slides, drivetrain, camera)
  }

  suspend fun place(level: Int, tolerance: Double = 0.5, condition: () -> Boolean = { true }){
    if(!camera.cameraRunning) throw IllegalStateException("Cannot run place script, camera is not running")
    if(camera.alignment != Webcam.Alignment.ALL) throw IllegalStateException("Cannot run place script, alignment mode ${camera.alignment} should be ALL")

    var lastClosest = camera.closest ?: return

    slides.apply{
      targetPosition = when(level){
        0 -> low
        1 -> medium
        2 -> high
        else -> 0.0
      }
    }

    val controller = PSVAController(constants)

    while (condition()){
      val closest = camera.closest ?: lastClosest
      lastClosest = closest
      
      val x = closest.xy
      if ((x.magnitude - 5.0) <= tolerance) break
      val targetVel = (closest.theta + 90.0.deg).dir * sqrt(2 * constants.maxAcceleration * abs(x.magnitude - 5.0))
      val v = drivetrain.relativeVel

      val tw = (v.y * x.x - v.x * x.y) * x.magnitude / x.y.pow(3.0)
      controller.update(Pose(targetVel * sign(x.magnitude - 5.0), tw))
      // println(constants.psva(controller.wheels, drivetrain.vel))
      println(controller.vel)

      drivetrain.move(constants.psva(controller.wheels * 1.0, drivetrain.vel))

      yield()
    }

    drivetrain.move(DriveValues.zero)
    slides.open()
  }


}
