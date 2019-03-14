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
open ANewDawn.Maneuver
open ANewDawn.Control
open ANewDawn.Math


let rescueFromLko () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let tgt = mission.SpaceCenter.TargetVessel

    printfn "Launching rescue mission towards %s" tgt.Name

    Launch.launch mission { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 140_000.<m> }
    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.Execution.executeNext mission 30.<s>
        
    let _intercept: Option<Node> = Maneuver.Interception.addVesselInterceptionNode mission tgt.Orbit (120.<s>, 0.5 * 6. * 60. * 60.<s>) (8. * 60.<s>, 24. * 60.<s>)

    Maneuver.Execution.executeNext mission 30.<s>

    let _catchUp = Maneuver.Interception.addCatchUpTargetNode mission tgt.Orbit
    Maneuver.Execution.executeNextExt mission 30.<s> { earlyStartFactor = 1.0 }

    mission.ActiveVessel.Control.Lights <- true

    printfn "Reached target - waiting for boarding. Press enter to resume flight..."
    let _ = Console.ReadLine()

    let _deorbit = Maneuver.Interception.addSurfaceInterceptionNode mission (-0.10266804865356<deg>, -74.575385655446<deg>) 45_000.<m> (60.<s>, mission.ActiveVessel.Orbit.Period.As<s>())
    Maneuver.Execution.executeNext mission 30.<s>
    
    printfn "Assume manual control!"
    
    
let rescueFromHko () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let tgt = mission.SpaceCenter.TargetVessel

    printfn "Launching rescue mission towards %s" tgt.Name

    let hkoProfile = { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 76_000.<m> }
    Launch.launchToInclination mission hkoProfile tgt.Orbit

    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Apoapsis

    Maneuver.Execution.executeNext mission 30.<s>
   
    let _intercept: Option<Node> = Maneuver.Interception.addVesselInterceptionNode mission tgt.Orbit (300.<s>, 12. * 6. * 60. * 60.<s>) (0.8 * 60. * 60.<s>, 1.5 * 6. * 60. * 60.<s>)
    Maneuver.Execution.executeNext mission 30.<s>

    let etaArrival = mission.ActiveVessel.Orbit.TimeOfClosestApproach(tgt.Orbit).As<s>() - mission.UniversalTime
    printfn "ETA to target encounter: %.0f" etaArrival

    printfn "Waiting until half-way correction"
    mission.WarpTo(mission.UniversalTime + etaArrival * 0.5)

    let etaArrival = mission.ActiveVessel.Orbit.TimeOfClosestApproach(tgt.Orbit).As<s>() - mission.UniversalTime
        
    let _halfWayCorrection: Option<Node> = Maneuver.Interception.addVesselInterceptionNode mission tgt.Orbit (0.1 * etaArrival, 0.9 * etaArrival) (0.98 * etaArrival, 1.02 * etaArrival)
    
    printfn "Confirm course correction..."
    let _ = Console.ReadLine()

    Maneuver.Execution.executeNext mission 30.<s>

    let _catchUp, burnTimeFactor = Maneuver.Interception.addCatchUpTargetNode mission tgt.Orbit
    printfn "F: %f" burnTimeFactor
    
    Maneuver.Execution.executeNextExt mission 30.<s> { earlyStartFactor = 1. }

    mission.ActiveVessel.Control.Lights <- true

    printfn "Reached target - waiting for boarding. Press enter to resume flight..."
    let _ = Console.ReadLine()
    
    let _deorbit = Maneuver.Interception.addSurfaceInterceptionNode mission (-0.10266804865356<deg>, -74.575385655446<deg> + 12.<deg>) 45_000.<m> (60.<s>, 6. * 3600.<s>)
    Maneuver.Execution.executeNext mission 30.<s>
    
    printfn "Assume manual control!"
    
let launchKerbinScanner () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let ship = mission.SpaceCenter.ActiveVessel

    printfn "Launching %s into polar orbit" ship.Name

    let polarProfile = { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; TargetApoapsis = 80_000.<m>; LaunchAzimuth = 0.<deg> }
    Launch.launch mission polarProfile

    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.Execution.executeNext mission 30.<s>
    
    let _transfer: Node = Maneuver.Orbital.addHohmannNode mission Apsis.Apoapsis 450_000.<m>
    Maneuver.Execution.executeNext mission 30.<s>

    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.Execution.executeNext mission 30.<s>

    ship.Control.Lights <- true // HACK: light switch toggles scanner

    printfn "Commencing scan"

    
let launchMunScanner () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let ship = mission.SpaceCenter.ActiveVessel

    printfn "Launching %s" ship.Name

    let munProfile = { Launch.KerbinProfile with MinTurnAlt = 500.<m>; MinTurnSpeed = 70.<m/s>; MaxQ = 20_000.<Pa> }
    Launch.launch mission munProfile 

    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Apoapsis
    Maneuver.Execution.executeNext mission 30.<s>
    
    let mun = mission.SpaceCenter.Bodies.["Mun"]
    
    let intercept: Option<Node> = Maneuver.Interception.addPlanetaryInterceptionNode mission mun (300.<s>, 2. * 6. * 60. * 60.<s>) (0.8 * 60. * 60.<s>, 1.5 * 6. * 60. * 60.<s>)
    
    Maneuver.Execution.executeNext mission 30.<s>

    let munOrbit = ship.Orbit.NextOrbit
    let etaMun = ship.Orbit.TimeToSOIChange.As<s>()

    printfn "ETA to Mun: %.0f s" etaMun

    mission.WarpTo(mission.UniversalTime + etaMun + 10.<s>)
    
    // NOTE: for polar orbit, place ship exactly in front/behind Mun's SOI at some vertical offset from the orbit itself

    let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Apsis.Periapsis
    Maneuver.Execution.executeNext mission 30.<s>
    
    ship.Control.Lights <- true // HACK: light switch toggles scanner

    ()

[<EntryPoint>]
let main argv = 
    // MinmusResearchVessel.run ()
    // rescueFromLko ()
    // MunLander.run ()
    let kerbinMu = 3.5316000e12<m^3/s^2>

    let sidereal = 21_549.425<s>
    let solar = 21_600.0<s>

    let rKerbin = 600.e3<m>

    printfn "%f" (Util.cbrt (Util.cube 13.))
    printfn "%.0f" (Kepler.semiMajorAxisFromPeriod kerbinMu sidereal - rKerbin)
    printfn "%.0f" (Kepler.semiMajorAxisFromPeriod kerbinMu solar - rKerbin)

    let sma = 2863.33e3<m> + rKerbin

    printfn "%.3f" (Kepler.orbitalPeriod kerbinMu sma)
    0