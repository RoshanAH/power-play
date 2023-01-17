package org.firstinspires.ftc.teamcode.robots


import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.roshanah.jerky.utils.DriveConstants
import com.roshanah.jerky.utils.DriveValues
import com.roshanah.jerky.utils.PSVAConstants
import kotlinx.coroutines.delay

import org.firstinspires.ftc.teamcode.components.*
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.readFile

class Ptoneigh(val active : () -> Boolean) : Robot(){

    lateinit var drivetrain : Mecanum;
    lateinit var turret : Turret;
    lateinit var claw : Claw;
    lateinit var camera : Webcam;
    lateinit var imu : IMU;


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
            PSVAConstants(0.0, 0.05, 0.013, 0.002),
            PSVAConstants(0.0, 0.01, 0.017, 0.001),
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

        camera = Webcam(map, "camera", "mount", { turret.slidePosition }, { drivetrain.relativeVel.y }).apply { kp = 0.05 }

        imu = IMU(map, "imu", jawsHubOrientation)

        drivetrain =
                Mecanum(map, "fl", "fr", "bl", "br", { voltage }).apply {
                    trackRadius = constants.trackRadius
                    ticksPerInch = 30.9861111
                }
        addComponents(turret, drivetrain, camera, imu, claw)
        TODO("Not yet implemented")

    }

    suspend fun closeAndRaise(){
        claw.close()
        delay(500L)
        turret.targetSlidePosition += 0.07

    }
}