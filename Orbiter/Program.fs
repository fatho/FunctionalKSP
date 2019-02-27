// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.

open FunctionalKSP

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System.Threading

open FunctionalKSP
open FunctionalKSP.Math
open FunctionalKSP.Units
open FunctionalKSP.Control
open System
open FunctionalKSP
open FunctionalKSP.Mission

let run (mission: Mission<unit>) =
    /// Make connection
    use conn = new Connection()
    Mission.run (MissionLoggers.stdoutLogger MissionLogLevel.Info) conn mission

let orbiter: Mission<unit> = Mission.mission {
    let! vessel = Mission.activeVessel
    // do! Rules.requireControllableVessel vessel
    do! Mission.missionPrimitive <| fun env -> Launch.launch env.ksc env.ksc.ActiveVessel Launch.KerbinProfile
    //do! Maneuver.circularize Apsis.Apoapsis 20.<s>
    //do! Maneuver.deorbit 300.<s>
}

//fun conn ksc ->
//    /// Launch into LKO
//    Launch.launch ksc ksc.ActiveVessel Launch.KerbinProfile

//    use clock = new Clock(conn, ksc)
//    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)
//    let circularization = Maneuver.addCircularizationNode clock ksc.ActiveVessel Maneuver.Apoapsis
//    Maneuver.executeNext clock steering 30.<s>

    
let steeringTest = Mission.mission {
    let! vessel = Mission.activeVessel
    //do Thread.Sleep(3000)

    //do! Mission.log Mission.Info "Spinning out of control"
    //do
    //    let rand = new Random()
    //    vessel.Control.Pitch <- float32 <| rand.NextDouble() * 2. - 1.
    //    vessel.Control.Yaw <- float32 <| rand.NextDouble() * 2. - 1.
    //    vessel.Control.Roll <- float32 <| rand.NextDouble() * 2. - 1.
    //    Thread.Sleep(3000)
    
    //do! Mission.log Mission.Info "Killing rotation"
    //let! residual = Maneuver.killRotation 0.05<deg/s> 30.<s>
    //return! Mission.log Mission.Info (sprintf "Residual rotation %.3f deg/s" (Vec3.mag residual))
    
    do! Mission.log Mission.Info "Orienting prograde"
    let! (residualAngle, residualVel) = Maneuver.orient Vec3.unitY vessel.OrbitalReferenceFrame 0.5<deg> 0.05<deg/s> 30.<s>
    return! Mission.log Mission.Info (sprintf "Residual rotation %.3f deg ; %.3f deg/s" residualAngle residualVel)
}
//    ()

//let hohmannTest() = withKSP <| fun conn ksc ->
//    use clock = new Clock(conn, ksc)
//    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)

//    Thread.Sleep(3000)
    
//    //let hohmannTransfer = Maneuver.addHohmannNode clock ksc.ActiveVessel Maneuver.Apoapsis 200_000.<m>
//    //Maneuver.executeNext clock steering
//    let circularization = Maneuver.addCircularizationNode clock ksc.ActiveVessel Maneuver.Apoapsis
//    Maneuver.executeNext clock steering 10.<s>
//    ()

[<EntryPoint>]
let main argv = 
    run orbiter

    //let up = Vec3.pack (0., 0., -1.)
    //let forward = Vec3.pack (0., 1., 0.)
    //let left = Vec3.pack (-1., 0., 0.)

    //let testRot fromV toV =
    //    printfn "Rotating %O to %O" fromV toV
    //    let axis, angle = Rotation.axisAngle fromV toV
    //    printfn "  %O %.1f" axis (angle * Units.degPerRad)
    //    let euler = Rotation.axisAngleToEuler axis angle * Units.degPerRad
    //    printfn "  %O" euler

    //testRot forward up
    //testRot forward left
    //testRot forward (Vec3.norm (left + forward))
    //testRot forward (Vec3.norm (left + up))
    //testRot forward -forward

    0