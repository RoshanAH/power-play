package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.abs
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.core.Component

class Mecanum(
    map: HardwareMap,
    fl: String,
    fr: String,
    bl: String,
    br: String,
) : Component {


  val fl = map.get(DcMotorEx::class.java, fl)
  val fr = map.get(DcMotorEx::class.java, fr)
  val bl = map.get(DcMotorEx::class.java, bl)
  val br = map.get(DcMotorEx::class.java, br)

  var ticksPerInch = 0.0
  var ticksPerDegree = 0.0

  var flPower: Double = 0.0
  var frPower: Double = 0.0
  var blPower: Double = 0.0
  var brPower: Double = 0.0

  var flp = 0.0
    private set
  var frp = 0.0
    private set
  var blp = 0.0
    private set
  var brp = 0.0
    private set

  var flv = 0.0
    private set
  var frv = 0.0
    private set
  var blv = 0.0
    private set
  var brv = 0.0
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

  fun moveTo(fltp: Double, frtp: Double, bltp: Double, brtp: Double, kp: Double) =
      move(
          (fltp - flp) * kp,
          (frtp - frp) * kp,
          (bltp - flp) * kp,
          (brtp - brp) * kp,
      )


  suspend fun moveUntilDone(
      fltp: Double,
      frtp: Double,
      bltp: Double,
      brtp: Double,
      kp: Double,
      tolerance: Double,
      update: () -> Unit = {}
  ) {

    while (abs(fltp - flp) > tolerance &&
        abs(frtp - frp) > tolerance &&
        abs(bltp - blp) > tolerance &&
        abs(brtp - brp) > tolerance) {
      moveTo(fltp, frtp, bltp, brtp, kp)
      update()
      yield()
    }
  }

  fun turnTo(theta: Double, kp: Double){
    val inchesPerDegree = ticksPerInch / ticksPerDegree
    val inches = theta * inchesPerDegree
    moveTo(inches, -inches, inches, -inches, kp * inchesPerDegree)
  } 

  suspend fun turnUntil(
    theta: Double,
    kp: Double,
    tolerance: Double,
    update: () -> Unit = { }
  ){
    val inchesPerDegree = ticksPerInch / ticksPerDegree
    val inches = theta * inchesPerDegree
    moveUntilDone(inches, -inches, inches, -inches, kp * inchesPerDegree, tolerance * inchesPerDegree, update)
  }

  fun forwardTo(inches: Double, kp: Double) = moveTo(inches, inches, inches, inches, kp)
  suspend fun forwardUntilDone(
    inches: Double,
    kp: Double,
    tolerance: Double,
    update: () -> Unit = {}
  ) = moveUntilDone(
    inches,
    inches,
    inches,
    inches,
    kp,
    tolerance,
    update
  )

  fun strafeTo(inches: Double, kp: Double) = moveTo(inches, -inches, -inches, inches, kp)
  suspend fun strafeUntilDone(
    inches: Double,
    kp: Double,
    tolerance: Double,
    update: () -> Unit = {}
  ) = moveUntilDone(
    inches,
    -inches,
    -inches,
    inches,
    kp,
    tolerance,
    update
  )
    

  fun drive(x: Double, y: Double, rot: Double) =
      move(
          y + x + rot,
          y - x - rot,
          y - x + rot,
          y + x - rot,
      )

  fun drive(gamepad: Gamepad) =
      drive(
          gamepad.left_stick_x.toDouble(),
          -gamepad.left_stick_y.toDouble(),
          gamepad.right_stick_x.toDouble()
      )

  override fun init(scope: CoroutineScope) {}

  override fun start(scope: CoroutineScope) {}

  override fun update(scope: CoroutineScope) {

    flp = fl.getCurrentPosition() / ticksPerInch
    frp = fr.getCurrentPosition() / ticksPerInch
    blp = bl.getCurrentPosition() / ticksPerInch
    brp = br.getCurrentPosition() / ticksPerInch

    flv = fl.getVelocity() / ticksPerInch
    frv = fr.getVelocity() / ticksPerInch
    blv = bl.getVelocity() / ticksPerInch
    brv = br.getVelocity() / ticksPerInch

    fl.setPower(flPower)
    fr.setPower(frPower)
    bl.setPower(blPower)
    br.setPower(brPower)
  }
}
