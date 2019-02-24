// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.

open FunctionalKSP

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System.Threading

open FunctionalKSP
open FunctionalKSP.LinearAlgebra
open FunctionalKSP.Units
open FunctionalKSP.Control
open System
open FunctionalKSP

let withKSP (f: Connection -> SpaceCenter.Service -> unit) =
    /// Make connection
    use conn = new Connection()
    let ksc = conn.SpaceCenter()
    Rules.requireControllableVessel ksc.ActiveVessel
    f conn ksc

let orbiter () = withKSP <| fun conn ksc ->
    use clock = new Clock(conn, ksc)
    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)
    /// Launch into LKO
    Launch.launch ksc ksc.ActiveVessel Launch.KerbinProfile
    let circularization = Maneuver.addCircularizationNode clock ksc.ActiveVessel Maneuver.Apoapsis
    Maneuver.executeNext clock steering 30.<s>

    
let steeringTest () = withKSP <| fun conn ksc ->
    use clock = new Clock(conn, ksc)
    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)
    
    Thread.Sleep(3000)

    printfn "Spinning out of control"
    let rand = new Random()
    ksc.ActiveVessel.Control.Pitch <- float32 <| rand.NextDouble() * 2. - 1.
    ksc.ActiveVessel.Control.Yaw <- float32 <| rand.NextDouble() * 2. - 1.
    ksc.ActiveVessel.Control.Roll <- float32 <| rand.NextDouble() * 2. - 1.
    
    //ksc.ActiveVessel.Control.Yaw <- 1.f

    Thread.Sleep(3000)
    
    printfn "Orienting to prograde"

    let prograde = Vec3.pack <| ksc.ActiveVessel.Flight(ksc.ActiveVessel.OrbitalReferenceFrame).Prograde
    steering.Mode <- Rocket.SteeringMode.LockTarget (prograde, ksc.ActiveVessel.OrbitalReferenceFrame)

    while true do
        let time = clock.Tick()
        steering.Update(time)
    ()

let hohmannTest() = withKSP <| fun conn ksc ->
    use clock = new Clock(conn, ksc)
    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)

    Thread.Sleep(3000)
    
    //let hohmannTransfer = Maneuver.addHohmannNode clock ksc.ActiveVessel Maneuver.Apoapsis 200_000.<m>
    //Maneuver.executeNext clock steering
    let circularization = Maneuver.addCircularizationNode clock ksc.ActiveVessel Maneuver.Apoapsis
    Maneuver.executeNext clock steering 10.<s>
    ()

[<EntryPoint>]
let main argv = 
    hohmannTest ()
    //steeringTest ()
    //// orbiter ()
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