package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.roshanah.jerky.math.Pose
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.profiling.buildProfile
import kotlin.math.abs
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.core.Component

class Mecanum(
    map: HardwareMap,
    fl: String,
    fr: String,
    bl: String,
    br: String,
    val voltage: () -> Double = { 12.0 }
) : Component {

  val fl = map.get(DcMotorEx::class.java, fl)
  val fr = map.get(DcMotorEx::class.java, fr)
  val bl = map.get(DcMotorEx::class.java, bl)
  val br = map.get(DcMotorEx::class.java, br)

  var ticksPerInch = 0.0
  var ticksPerDegree = 0.0
  var trackWidth = 0.0

  var flPower: Double = 0.0
  var frPower: Double = 0.0
  var blPower: Double = 0.0
  var brPower: Double = 0.0

  val powers: DriveValues
    get() = DriveValues(flPower, frPower, blPower, brPower)

  var pos = DriveValues.zero
    private set

  var vel = DriveValues.zero
    private set

  var relativeVel = Pose.zero
    private set

  init {
    reset()

    this.fl.setDirection(DcMotorSimple.Direction.FORWARD)
    this.fr.setDirection(DcMotorSimple.Direction.REVERSE)
    this.bl.setDirection(DcMotorSimple.Direction.FORWARD)
    this.br.setDirection(DcMotorSimple.Direction.REVERSE)

    this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
  }

  fun move(fl: Double, fr: Double, bl: Double, br: Double) {
    var max = abs(fl)
    max = max.coerceAtLeast(abs(fr))
    max = max.coerceAtLeast(abs(bl))
    max = max.coerceAtLeast(abs(br))

    val scale = (1.0 / max).coerceAtMost(1.0)

    flPower = fl * scale
    frPower = fr * scale
    blPower = bl * scale
    brPower = br * scale
  }

  fun move(powers: DriveValues) = move(powers.fl, powers.fr, powers.bl, powers.br)

  fun reset() {
    fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

    fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
  }

  fun drive(x: Double, y: Double, rot: Double) =
      move(
          y + x + rot,
          y - x - rot,
          y - x + rot,
          y + x - rot,
      )

  fun drive(gamepad: Gamepad, scale: Double = 1.0, turnScale: Double = 1.0) =
    drive(
      gamepad.left_stick_x.toDouble() * scale,
      -gamepad.left_stick_y.toDouble() * scale,
      gamepad.right_stick_x.toDouble() * turnScale
    )

  override fun init(scope: CoroutineScope) {}

  override fun start(scope: CoroutineScope) {}

  override fun update(scope: CoroutineScope) {
    val voltage = voltage()

    pos =
        DriveValues(
            fl.getCurrentPosition().toDouble(),
            fr.getCurrentPosition().toDouble(),
            bl.getCurrentPosition().toDouble(),
            br.getCurrentPosition().toDouble()
        ) / ticksPerInch

    vel =
        DriveValues(fl.getVelocity(), fr.getVelocity(), bl.getVelocity(), br.getVelocity()) /
            ticksPerInch

    relativeVel =
      vel.run {
        Pose(
          (fl - fr - bl + br) * 0.25,
          (fl + fr + bl + br) * 0.25,
          ((-fl + fr - bl + br) / (4 * trackWidth)))
      }

    fl.setPower(flPower * 12.0 / voltage)
    fr.setPower(frPower * 12.0 / voltage)
    bl.setPower(blPower * 12.0 / voltage)
    br.setPower(brPower * 12.0 / voltage)
  }
}
