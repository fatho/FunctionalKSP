// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.

open FunctionalKSP

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System.Threading

open FunctionalKSP.LinearAlgebra
open FunctionalKSP
open FunctionalKSP.Control
open System
open FunctionalKSP.Control.Rocket

let withKSP (f: Connection -> SpaceCenter.Service -> unit) =
    /// Make connection
    use conn = new Connection()
    let ksc = conn.SpaceCenter()
    Rules.requireControllableVessel ksc.ActiveVessel
    f conn ksc

let orbiter () = withKSP <| fun conn ksc ->
    /// Launch into LKO
    Launch.launch ksc ksc.ActiveVessel Launch.KerbinProfile
    let circularization = Maneuver.addCircularizationNode ksc ksc.ActiveVessel Maneuver.Apoapsis
    Maneuver.executeNext ksc.ActiveVessel

    
let steeringTest () = withKSP <| fun conn ksc ->
    use clock = new Clock(conn, ksc)
    let steering = new Rocket.Steering(ksc, ksc.ActiveVessel)
    
    //Thread.Sleep(3000)

    //printfn "Applying control input"
    //let rand = new Random()
    //ksc.ActiveVessel.Control.Pitch <- float32 <| rand.NextDouble() * 2. - 1.
    //ksc.ActiveVessel.Control.Yaw <- float32 <| rand.NextDouble() * 2. - 1.
    //ksc.ActiveVessel.Control.Roll <- float32 <| rand.NextDouble() * 2. - 1.
    
    //Thread.Sleep(3000)
    let prograde = Vec3.pack <| ksc.ActiveVessel.Flight(ksc.ActiveVessel.OrbitalReferenceFrame).Prograde
    steering.Mode <- SteeringMode.LockTarget (prograde, ksc.ActiveVessel.OrbitalReferenceFrame)

    while true do
        let time = clock.Tick()
        steering.Update(time)
    ()

[<EntryPoint>]
let main argv = 
    steeringTest ()
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