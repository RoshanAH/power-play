package org.firstinspires.ftc.teamcode.robots

import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.components.Mecanum
import org.firstinspires.ftc.teamcode.components.Slides
import org.firstinspires.ftc.teamcode.components.Webcam
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ReadWriteFile
import kotlinx.coroutines.yield
import kotlin.math.abs
import kotlin.math.sign


class Jaws : Robot() {

  lateinit var drivetrain: Mecanum
  lateinit var slides: Slides
  lateinit var camera: Webcam

  val low = 0.45
  val medium = 0.7
  val high = 1.0

  val kAlignmentOmega = 0.0001
  val kAlignmentY = 0.01

  val alliance: String
  val side: String

  init {
    val allianceFile = AppUtil.getInstance().getSettingsFile("alliance.txt")
    val sideFile = AppUtil.getInstance().getSettingsFile("side.txt")

    alliance = ReadWriteFile.readFile(allianceFile) 
    side = ReadWriteFile.readFile(sideFile) 
  }

  override fun mapHardware(map: HardwareMap) {
    camera = Webcam(map, "camera", "mount"){
      (drivetrain.flv + drivetrain.frv + drivetrain.blv + drivetrain.brv) * 0.25
    }.apply{
      kp = 0.01
    }

    drivetrain = Mecanum(map, "fl", "fr", "bl", "br").apply {
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

  suspend fun alignAndPlace(distance: Double){
    slides.targetPosition = 1.0
    camera.aligning = true
    while (slides.position < 0.5) yield()
    while(camera.dist > distance) {
      val center = camera.alignmentPipeline.center
      if (center != null && camera.aligning) {
        val vertAlignment = (160.0 - abs(center.x)) / 160.0
        val forwardVel = (camera.dist - distance) * kAlignmentY * vertAlignment
        drivetrain.drive(
            0.0,
            abs(forwardVel).coerceIn(0.2 * vertAlignment, 0.4) * sign(forwardVel),
            center.x * kAlignmentOmega * camera.dist.coerceAtMost(6.0)
        )
      } else {
        drivetrain.drive(0.0, 0.0, 0.0)
      }
      yield()
    }

    slides.open()
  }

}
