module MunLander


open KRPC.Client
open ANewDawn.Mission
open ANewDawn.Units
open ANewDawn
open KRPC.Client.Services.SpaceCenter
open System
open ANewDawn.Control
open ANewDawn.Math
open ANewDawn.Control.AttitudeControl

let run () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let ship = mission.SpaceCenter.ActiveVessel
    let mun = mission.SpaceCenter.Bodies.["Mun"]

    /// Launch
    let phase1 () =
        printfn "Launching %s into LKO (80 km)" ship.Name
        Launch.launch mission { Launch.KerbinProfile with AttitudeControlProfile = AttitudeControlProfile.Heavy }
                
        printfn "Extending solar panels"
        ship.Control.SolarPanels <- true

        printfn "Circularizing..."
        let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Maneuver.Apoapsis
        Maneuver.Execution.executeNext mission 30.<s>
    
    /// Transfer burn
    let phase2 () =
        printfn "Plotting Mun course..."
        let _node = Maneuver.Interception.addPlanetaryInterceptionNode mission mun (120.<s>, 3600.<s>) (6. * 3600.<s>, 12. * 3600.<s>)
        printfn "Make manual adjustments and press enter to confirm!"
        let _ = Console.ReadLine()
        Maneuver.Execution.executeNext mission 30.<s>
        
    /// Mid-course correction
    let phase3 () =
        printfn "Waiting for mid-course correction"
        let eta1 = ship.Orbit.TimeToSOIChange.As<s>()

        let timeForCorrection = eta1 / 2.
        mission.WarpTo(mission.UniversalTime + timeForCorrection)

        printfn "Enter manual course-correction and press enter to confirm!"
        let _ = Console.ReadLine()
        while ship.Control.Nodes.Count > 0 do
            Maneuver.Execution.executeNext mission 30.<s>
        
        printfn "Warping to SOI change, press enter to confirm!"
        let _ = Console.ReadLine()

        let eta = ship.Orbit.TimeToSOIChange.As<s>()
        printfn "%s ETA: %.0f s" mun.Name eta
        mission.WarpTo(mission.UniversalTime + eta)
        
    /// Target orbit insertion
    let phase4 () =
        let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Maneuver.Periapsis
        Maneuver.Execution.executeNext mission 30.<s>
         
    /// Manual control for landing and return
    let phase5 () =
        printfn "Reached %s orbit, assume manual control!" mun.Name

        let mutable commandMode = true

        while commandMode do
            printfn "Enter command: "
            let command = Console.ReadLine()
            match command with
            | "cont" -> commandMode <- false
            | "exec" -> Maneuver.Execution.executeNext mission 30.<s>
            | "execearly" -> Maneuver.Execution.executeNextExt mission 30.<s> { earlyStartFactor = 1.0 }
            | "intercept" -> do
                let target = mission.SpaceCenter.TargetVessel
                if target = null then
                    printfn "No target!"
                else
                    let orbit = ship.Orbit
                    let tMax =
                        if orbit.NextOrbit <> null then
                            orbit.TimeToSOIChange.As<s>()
                        else
                            Kepler.orbitalPeriod (floatU <| orbit.Body.GravitationalParameter.As<m^3/s^2>()) (orbit.SemiMajorAxis.As<m>())
                    let node: Option<Node> = Maneuver.Interception.addVesselInterceptionNode mission target.Orbit (120.<s>, tMax) (120.<s>, tMax)
                    if node.IsNone then
                        printfn "No interception possible"
            | "catch" -> do
                let target = mission.SpaceCenter.TargetVessel
                if target = null then
                    printfn "No target!"
                else
                    Maneuver.Interception.addCatchUpTargetNode mission target.Orbit |> ignore
            | _ -> printfn "Unknown command"
        
        printfn "Enter return maneuver and press enter to confirm!"
        let _ = Console.ReadLine()
        Maneuver.Execution.executeNext mission 30.<s>
        
        printfn "Warping to SOI change, press enter to confirm!"
        let _ = Console.ReadLine()

        let eta = ship.Orbit.TimeToSOIChange.As<s>()
        printfn "%s ETA: %.0f s" mun.Name eta
        mission.WarpTo(mission.UniversalTime + eta)

    let phase6 () =
        printfn "Waiting for atmosphere"

        let flight = ship.Flight()
        use altitudeS = mission.Streams.UseStream<m>(fun () -> flight.SurfaceAltitude)

        while altitudeS.Value > 70_000.<m> do
            mission.Tick() |> ignore
        
        printfn "Entered atmosphere - orienting retrograde"
                
        AttitudeControl.loop mission ship.SurfaceVelocityReferenceFrame <| seq {

            while altitudeS.Value > 1_500.<m> do
                yield { forward = - Vec3.unitY; top = None }
            
        }

        printfn "Opening parachutes"
        ship.Control.Parachutes <- true


    //phase1 ()
    //phase2 ()
    //phase3 ()
    //phase4 ()
    phase5 ()
    // phase6 ()