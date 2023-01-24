package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.core.Component
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.rad
import kotlin.math.abs
import kotlin.math.sign

class Turret(map: HardwareMap, turret: String, slides: String) : Component {
    var maxticks = 2215
    var pS = 16.0
    var gravity = 0.07

    var pT = 0.01
    var dT = 0.0
    var fT = 0.2

    var thetaTolerance = 0.5
    var ticksPerDegree = 4.26666
    var maxTurretPower = 0.6
    var maxRotation = 270.0;

    val turret = map.dcMotor.get(turret);
    val slides = map.dcMotor.get(slides);

    init {
        this.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        this.slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        this.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        this.slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        this.turret.setDirection(DcMotorSimple.Direction.REVERSE)
        this.slides.setDirection(DcMotorSimple.Direction.REVERSE)
    }

    var height = 0.0
        private set

    var lowestRotationHeight = 0.3

    var targetHeight = 0.0
        set(value) {
          field = value.coerceIn(0.0..1.0)
        }

    private val slideError
        get() = targetHeight - height

    var theta = 0.0
        private set

    var targetTheta = 0.0
        set(value) {
            if(height + 0.05 > lowestRotationHeight){
              field = value.coerceIn(-270.0..270.0)
            }
        }
    private val turretError
        get() = targetTheta - theta


    override fun init(scope: CoroutineScope) {
    }

    override fun start(scope: CoroutineScope) {
    }

    override fun update(scope: CoroutineScope) {

      val rawHeight = slides.getCurrentPosition()
      val rawTheta = turret.getCurrentPosition()

      height = rawHeight.toDouble() / maxticks
      theta = rawTheta.toDouble() / ticksPerDegree

      val errorSlides = targetHeight.coerceAtLeast(if (abs(theta) > 1.5) lowestRotationHeight else 0.0) - height
      val errorTurret = targetTheta - theta

      val powerSlides = errorSlides * pS + if (targetHeight > 0.05) gravity else 0.0
      val powerTurret = errorTurret * pT + if(abs(errorTurret) < thetaTolerance) 0.0 else sign(errorTurret) * fT
      
      slides.setPower(powerSlides)
      turret.setPower(powerTurret.coerceIn(-maxTurretPower, maxTurretPower))
    }
}
