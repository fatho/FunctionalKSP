// Learn more about F# at http://fsharp.org
// See the 'F# Tutorial' project for more help.

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter

open System.Threading
open System

open ANewDawn.Math
open ANewDawn.Units
open ANewDawn.Mission
open ANewDawn
open ANewDawn.Extensions
open ANewDawn.Math.Lambert
open ANewDawn.Math.Kepler


let rescueFromLko () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let tgt = mission.SpaceCenter.TargetVessel

    printfn "Launching rescue mission towards %s" tgt.Name

    Launch.launch mission { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 120_000.<m> }
    let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.executeNext mission 30.<s>
        
    let _intercept: Node = Maneuver.addInterceptionNode mission tgt.Orbit (120.<s>, 6. * 60. * 60.<s>) (1. * 60.<s>, 20. * 60.<s>)

    Maneuver.executeNext mission 30.<s>

    let _catchUp = Maneuver.addCatchUpTargetNode mission tgt.Orbit
    Maneuver.executeNext mission 30.<s>

    mission.ActiveVessel.Control.Lights <- true

    printfn "Reached target - waiting for boarding. Press enter to resume flight..."
    let _ = Console.ReadLine()

    let _transfer: Node = Maneuver.addHohmannNode mission Apsis.Periapsis 80_000.<m>
    Maneuver.executeNext mission 30.<s>
    
    let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Periapsis
    Maneuver.executeNext mission 30.<s>

    let _deorbit = Maneuver.addDeorbitNode mission (-0.10266804865356<deg>, -74.575385655446<deg>) 45_000.<m> (2. * 3600.<s>)
    Maneuver.executeNext mission 30.<s>
    
    printfn "Assume manual control!"
    
    
let rescueFromHko () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let tgt = mission.SpaceCenter.TargetVessel

    printfn "Launching rescue mission towards %s" tgt.Name

    let hkoProfile = { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 76_000.<m> }
    Launch.launchToInclination mission hkoProfile tgt.Orbit

    let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Apoapsis

    Maneuver.executeNext mission 30.<s>
   
    let _intercept: Node = Maneuver.addInterceptionNode mission tgt.Orbit (300.<s>, 12. * 6. * 60. * 60.<s>) (0.8 * 60. * 60.<s>, 1.5 * 6. * 60. * 60.<s>)
    Maneuver.executeNext mission 30.<s>

    let etaArrival = mission.ActiveVessel.Orbit.TimeOfClosestApproach(tgt.Orbit).As<s>() - mission.UniversalTime
    printfn "ETA to target encounter: %.0f" etaArrival

    printfn "Waiting until half-way correction"
    mission.WarpTo(mission.UniversalTime + etaArrival * 0.5)

    let etaArrival = mission.ActiveVessel.Orbit.TimeOfClosestApproach(tgt.Orbit).As<s>() - mission.UniversalTime
        
    let _halfWayCorrection: Node = Maneuver.addInterceptionNode mission tgt.Orbit (0.1 * etaArrival, 0.9 * etaArrival) (0.98 * etaArrival, 1.02 * etaArrival)
    
    printfn "Confirm course correction..."
    let _ = Console.ReadLine()

    Maneuver.executeNext mission 30.<s>

    let _catchUp, burnTimeFactor = Maneuver.addCatchUpTargetNode mission tgt.Orbit
    printfn "F: %f" burnTimeFactor
    
    Maneuver.executeNextExt mission 30.<s> { earlyStartFactor = 1. }

    mission.ActiveVessel.Control.Lights <- true

    printfn "Reached target - waiting for boarding. Press enter to resume flight..."
    let _ = Console.ReadLine()
    
    let _deorbit = Maneuver.addDeorbitNode mission (-0.10266804865356<deg>, -74.575385655446<deg> + 12.<deg>) 45_000.<m> (6. * 3600.<s>)
    Maneuver.executeNext mission 30.<s>
    
    printfn "Assume manual control!"
    
let launchKerbinScanner () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let ship = mission.SpaceCenter.ActiveVessel

    // Maneuver.suicideBurn mission
    printfn "Launching %s into polar orbit" ship.Name

    let polarProfile = { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 80_000.<m>; LaunchAzimuth = 0.<deg> }
    Launch.launch mission polarProfile

    let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.executeNext mission 30.<s>
    
    let _transfer: Node = Maneuver.addHohmannNode mission Apsis.Apoapsis 450_000.<m>
    Maneuver.executeNext mission 30.<s>

    let _circ: Node = Maneuver.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.executeNext mission 30.<s>

    ship.Control.Lights <- true // HACK: light switch toggles scanner

    printfn "Commencing scan"


[<EntryPoint>]
let main argv = 
    launchKerbinScanner ()
    0