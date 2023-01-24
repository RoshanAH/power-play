package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.HardwareMap
import com.roshanah.jerky.math.Angle
import com.roshanah.jerky.math.rad
import kotlin.math.PI
import kotlin.math.asin
import kotlin.math.pow
import kotlin.math.sqrt
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.core.Component

class Claw(map: HardwareMap, claw: String, extension: String) : Component {
  var open = 0.0
  var close = 0.25
  var clawPos = open
    private set

  var servoHeight = 1.139 // height above linear slides
  var l1 = 8.18898
  var l2 = 9.76378

  var extendedPos = 0.48
  var extendedLength = 11.5
  var retractedPos = 0.7
  var retractedLength = 3.125

  val maxExtension: Double
    get() = extendedLength - retractedLength

  var targetExtension = 0.0
    set(value) {
      field = value.coerceIn(0.0..extendedLength)
    }

  val claw = map.servo.get(claw)
  val extension = map.servo.get(extension)

  fun open() {
    clawPos = open
  }

  fun close() {
    clawPos = close
  }

  override fun init(scope: CoroutineScope) {
    claw.setPosition(clawPos)
    extension.setPosition(retractedPos)
  }

  override fun start(scope: CoroutineScope) {}

  override fun update(scope: CoroutineScope) {
    claw.setPosition(clawPos)
    val retractedAngle = solveForAngle(retractedLength)
    val extendedAngle = solveForAngle(extendedLength)
    val targetAngle = solveForAngle(targetExtension + retractedLength)

    val scaledExtension = (targetAngle - retractedAngle).rad / (extendedAngle - retractedAngle).rad
    extension.setPosition(retractedPos + (extendedPos - retractedPos) * scaledExtension)
  }

  private fun solveForAngle(
      extension: Double
  ): Angle { // uses inverse kinematics of a 2 joint linkage powered slide
    val llwh = (l2.pow(2.0) - l1.pow(2.0) - extension.pow(2.0) - servoHeight.pow(2.0)) / (2 * l1)
    val a = extension.pow(2.0) + servoHeight.pow(2.0)
    val b = servoHeight * llwh
    val c = llwh.pow(2.0) - extension.pow(2.0)
    val sinTheta = (b + sqrt(b.pow(2.0) - a * c)) / a
    val minExtension = sqrt(l2.pow(2.0) - (servoHeight + l1).pow(2.0))
    return if (minExtension < extension) asin(sinTheta).rad else (PI - asin(sinTheta)).rad
  }
}
