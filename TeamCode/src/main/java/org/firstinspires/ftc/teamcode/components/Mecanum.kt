package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.Component;

class Mecanum(map: HardwareMap, fl: String, fr: String, bl: String, br: String) : Component{
  val fl = map.dcMotor.get(fl);
  val fr = map.dcMotor.get(fr);
  val bl = map.dcMotor.get(bl);
  val br = map.dcMotor.get(br);

  var flPower: Double = 0.0;
  var frPower: Double = 0.0;
  var blPower: Double = 0.0;
  var brPower: Double = 0.0;

  init {
    this.fl.setDirection(DcMotorSimple.Direction.FORWARD);
    this.fr.setDirection(DcMotorSimple.Direction.REVERSE);
    this.bl.setDirection(DcMotorSimple.Direction.FORWARD);
    this.br.setDirection(DcMotorSimple.Direction.REVERSE);

    this.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  fun move(fl: Double, fr: Double, bl: Double, br: Double) {
    flPower = fl;
    frPower = fr;
    blPower = bl;
    brPower = br;
  }

  fun drive(x: Double, y: Double, rot: Double) = move(
    y + x + rot,
    y - x - rot,
    y - x + rot,
    y + x - rot,
  )

  fun drive(gamepad: Gamepad) = drive(
    gamepad.left_stick_x.toDouble(), 
    -gamepad.left_stick_y.toDouble(), 
    gamepad.right_stick_x.toDouble()
  )

  override fun init() {}

  override fun start() {}

  override fun update() {
    fl.setPower(flPower);
    fr.setPower(frPower);
    bl.setPower(blPower);
    br.setPower(brPower);
  }
}
