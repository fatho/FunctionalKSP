module MinmusResearchVessel

open KRPC.Client
open ANewDawn.Mission
open ANewDawn.Units
open ANewDawn
open KRPC.Client.Services.SpaceCenter
open System
open ANewDawn.Control
open ANewDawn.Math

let run () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let ship = mission.SpaceCenter.ActiveVessel
    let minmus = mission.SpaceCenter.Bodies.["Minmus"]

    /// Launch
    let phase1 () =
        printfn "Acquiring target lock"
        mission.SpaceCenter.TargetBody <- minmus

        printfn "Launching %s into %s inclination (%.1f deg)" ship.Name minmus.Name (minmus.Orbit.Inclination * degPerRad)
        Launch.launchToInclination mission Launch.KerbinProfile minmus.Orbit
                
        printfn "Extending solar panels"
        ship.Control.SolarPanels <- true

        printfn "Circularizing..."
        let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Maneuver.Apoapsis
        Maneuver.Execution.executeNext mission 30.<s>
    
    /// Transfer burn
    let phase2 () =
        printfn "Plotting Minmus course..."
        let _node = Maneuver.Interception.addPlanetaryInterceptionNode mission minmus (120.<s>, 3600.<s>) (45. * 3600.<s>, 62. * 3600.<s>)
        printfn "Press enter to confirm!"
        let _ = Console.ReadLine()
        //_node.Value.Remove()
        Maneuver.Execution.executeNext mission 30.<s>
        
    /// Course correction
    let phase3 () =
        printfn "Enter manual course-correction and press enter to confirm!"
        let _ = Console.ReadLine()
        Maneuver.Execution.executeNext mission 30.<s>
        
        printfn "Warping to SOI change, press enter to confirm!"
        let _ = Console.ReadLine()

        let eta = ship.Orbit.TimeToSOIChange.As<s>()
        printfn "%s ETA: %.0f s" minmus.Name eta
        mission.WarpTo(mission.UniversalTime + eta)
        
    /// Target orbit insertion
    let phase4 () =
        let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Maneuver.Periapsis
        Maneuver.Execution.executeNext mission 30.<s>
         
    /// Execute return maneuver
    let phase5 () =
        printfn "Enter manual course-correction and press enter to confirm!"
        let _ = Console.ReadLine()
        Maneuver.Execution.executeNext mission 30.<s>

    /// Oh s**t, I didn't turn off the oven
    let phase6 () =
        //printfn "Plotting emergency return course...."
        //let _deorbit = Maneuver.Orbital.addDeorbitNode mission (-0.10266804865356<deg>, -74.575385655446<deg> + 24.<deg>) 45_000.<m> (120.<s>, 6. * 3600.<s>)
        //printfn "Press enter to confirm!"
        //let _ = Console.ReadLine()
        //Maneuver.Execution.executeNext mission 30.<s>

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


    // phase1 ()
    // phase2 ()
    // phase3 ()
    // phase4 ()
    // phase5 ()
    phase6 ()