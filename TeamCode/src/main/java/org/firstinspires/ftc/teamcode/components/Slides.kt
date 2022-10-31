package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.core.Component

class Slides(map: HardwareMap, left: String, right: String, claw: String) : Component {

  var maxticks = 0
  var p = 0.0
  var d = 0.0
  var gravity = 0.0

  var open = 0.5
  var close = 0.5
  var clawPos = open

  val left = map.dcMotor.get(left)
  val right = map.dcMotor.get(right)
  val claw = map.servo.get(claw)

  init {
    this.right.setDirection(DcMotorSimple.Direction.FORWARD)
    this.left.setDirection(DcMotorSimple.Direction.REVERSE)
  }

  var position = 0.0
    private set
  var targetPosition = 0.0
    set(value) {
      field = value.coerceIn(0.0..1.0)
    }

  private var lastTime = System.nanoTime() * 1e-9
  private var lastError = 0.0

  private val error
    get() = targetPosition - position


  override fun init() {
    left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)  
    right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)  

    left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)  
    right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)  
  }
  override fun start() {}
  override fun update() {


    val time = System.nanoTime() * 1e-9
    val deltaTime = time - lastTime
    lastTime = time

    position = (left.getCurrentPosition() + right.getCurrentPosition()).toDouble() * 0.5 / maxticks
    val derivative = -error / deltaTime

    val power = error * p + derivative * d + if (targetPosition > 0.05) gravity else 0.0

    left.setPower(power)
    right.setPower(power)

    claw.setPosition(clawPos)

    lastError = error 

  }
}
