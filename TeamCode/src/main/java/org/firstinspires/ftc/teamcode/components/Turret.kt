package org.firstinspires.ftc.teamcode.components

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.CoroutineScope
import org.firstinspires.ftc.teamcode.core.Component
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DcMotor
import com.roshanah.jerky.math.deg
import com.roshanah.jerky.math.rad

class Turret(map: HardwareMap, turret: String, slides: String) : Component {
    var maxticks = 0
    var pS = 0.0
    var dS = 0.0
    var gravity = 0.0

    var pT = 0.0
    var dT = 0.0
    var maxRotation = 270.0;
    var tpd = 0;

    val turret = map.dcMotor.get(turret);
    val slides = map.dcMotor.get(slides);

    init {
        this.turret.setDirection(DcMotorSimple.Direction.REVERSE)
        this.slides.setDirection(DcMotorSimple.Direction.REVERSE)
    }
    var slidePosition = 0.0
        private set
    var targetSlidePosition = 0.0
        set(value) {
            field = value.coerceIn(0.0..1.0)
        }
    private var lastTime = System.nanoTime() * 1e-9
    private var lastError = 0.0

    private val slideError
        get() = targetSlidePosition - slidePosition

    var theta = 0.0
        private set
    var targetTheta = 0.0
        set(value) {
            field = value.coerceIn(-270.0..270.0)
        }
    private val turretError
        get() = targetTheta - theta


    override fun init(scope: CoroutineScope) {
        TODO("Not yet implemented")
    }

    override fun start(scope: CoroutineScope) {
        TODO("Not yet implemented")
    }

    override fun update(scope: CoroutineScope) {
        TODO("Not yet implemented")
    }


}