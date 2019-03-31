module SatelliteConstellation

open KRPC.Client
open ANewDawn.Mission
open ANewDawn.Units
open ANewDawn
open KRPC.Client.Services.SpaceCenter
open System
open ANewDawn.Control
open ANewDawn.Math
open System.Threading
open System.Resources

let run () =
    use conn = new Connection()
    use mission = new Mission(conn)
    let deployShip = mission.SpaceCenter.ActiveVessel
    let kerbin = mission.SpaceCenter.Bodies.["Kerbin"]
    let kerbinMu = floatU <| kerbin.GravitationalParameter.As<m^3/s^2>()

    // ======= Compute orbits =======

    let numSatellites = 4

    // Compute the parameters of the keostationary orbit (where the orbital period is equal to a sidereal day)
    let lengthOfDay = floatU <| kerbin.RotationalPeriod.As<s>()
    let kerbinRadius = floatU <| kerbin.EquatorialRadius.As<m>()
    // This is the semi-major axis we want to achieve. Orbit doesn't have to be perfectly circular, as long as the SMA is correct
    let keostationarySma = Kepler.semiMajorAxisFromPeriod kerbinMu lengthOfDay
    // For a circular orbit, the SMA is simply the distance from the center of mass of the central body
    let keostationaryAltitude = keostationarySma - kerbinRadius

    // For the deployment orbit, we want to place each satellite 1 / numSatellites of the orbit before the previous one,
    let deployPeriod = lengthOfDay * (float numSatellites - 1.) / float numSatellites
    let deploySma = Kepler.semiMajorAxisFromPeriod kerbinMu deployPeriod
    // The deployment apoapsis must touch the keostationary orbit
    let deployApoapsis = keostationaryAltitude
    // Matching periapsis for getting the desired SMA
    let deployPeriapsis = 2. * deploySma - deployApoapsis - 2. * kerbinRadius

    printfn "Deploying %d Satellite Constellation at %f km" numSatellites (keostationaryAltitude / 1000.)
    printfn "SMA: %.3f" keostationarySma
    printfn "Deploy Periapsis: %.0f m" deployPeriapsis

    /// Launch
    let phase1 () =
        printfn "Launching %s into equatorial LKO" deployShip.Name
        Launch.launch mission Launch.KerbinProfile

        // We reached space here, jettison fairings while still suborbital
        for fairing in deployShip.Parts.Fairings do
            fairing.Jettison()
        
        // Open solar panels of launcher (but not those in the satellites, therefore stopping at decouplers)
        let rec openSolarPanels (root: Part) =
            let panel = root.SolarPanel
            if panel <> null then
                if panel.Deployable then
                    panel.Deployed <- true
            elif root.Decoupler <> null then
                for child in root.Children do
                    openSolarPanels child
        openSolarPanels deployShip.Parts.Root
        
        printfn "Circularizing..."
        deployShip.Control.RCS <- true
        let _circ: Node = Maneuver.Orbital.addCircularizationNode mission Maneuver.Apoapsis
        Maneuver.Execution.executeNext mission 30.<s>
    
    /// Reach deployment orbit
    let phase2 () =
        deployShip.Control.RCS <- true
        printfn "Increasing apoapsis to keostationary orbit"
        let _node = Maneuver.Orbital.addHohmannNode mission Maneuver.Apoapsis deployApoapsis
        Maneuver.Execution.executeNext mission 30.<s>
        
        printfn "Increasing periapsis to reach deployment orbit"
        let _node = Maneuver.Orbital.addHohmannNode mission Maneuver.Periapsis deployPeriapsis
        Maneuver.Execution.executeNext mission 30.<s>
    
    /// Deploy satellites
    let phase3 () =
        let positionSatellite satVessel =
            // switch to satellite
            mission.ActiveVessel <- satVessel
            Thread.Sleep 500

            printfn "  Taken control of satellite"
            
            printfn "  Extending solar panels"
            satVessel.Control.SolarPanels <- true
            
            printfn "  Deploying antennae"
            // activate antennae
            for antenna in satVessel.Parts.Antennas do
                if antenna.Deployable then
                    antenna.Deployed <- true

            printfn "  Finalizing orbit"

            // raise periapsis
            let _node = Maneuver.Orbital.addHohmannNode mission Maneuver.Periapsis keostationaryAltitude
            Maneuver.Execution.executeNext mission 30.<s>

            
            // fine tune
            let satOrbit = satVessel.Orbit
            use satSma = mission.Streams.UseStream<m>(fun () -> satOrbit.SemiMajorAxis)
            printfn "  Achieved SMA: %.5f" satSma.Value
            printfn "  Desired SMA : %.5f" keostationarySma

            if abs (satSma.Value - keostationarySma) >= 0.1<m> then
                printfn "  Fine tuning orbit"
                let direction = float (sign (keostationarySma - satSma.Value))
                let correctionDir = { forward = direction * Vec3.unitY; top = None }

                Maneuver.Execution.orient mission satVessel.OrbitalReferenceFrame correctionDir.forward 0.5<deg> 0.1<deg/s> 60.<s> |> ignore

                AttitudeControl.loop mission satVessel.OrbitalReferenceFrame <| seq {
                    while direction * satSma.Value < direction * keostationarySma do
                        let throttle = min 0.1 <| abs (satSma.Value - keostationarySma) / 10000.<m>
                        satVessel.Control.Throttle <- float32 throttle
                        yield correctionDir
                }
                satVessel.Control.Throttle <- 0.f
                printfn "  Done"
                printfn "  Achieved SMA: %.5f" satSma.Value

            // switch back to deployment ship
            mission.ActiveVessel <- deployShip
            Thread.Sleep 500

        let deployNextSatellite num = 
            let satName = sprintf "KEOCOMM v1 - %d" num

            printfn "  Deploying satellite #%d" num
            let vessels = deployShip.Control.ActivateNextStage()

            printfn "  Waiting for clearance"
            mission.WarpTo(mission.UniversalTime + 60.<s>)

            if vessels.Count = 0 then
                printfn "  Deployment failed"
            else
                for sat in vessels do
                    if sat.Type = VesselType.Debris then
                        printfn "  Found debris, ignoring"
                    else
                        printfn "  Found satellite, positioning"
                        sat.Name <- satName
                        positionSatellite sat

        let deployOrbit = deployShip.Orbit

        for satNum in {1..numSatellites} do
            let now = mission.UniversalTime
            let timeToApoapsis = deployOrbit.TimeToApoapsis.As<s>()
            let nextDeploymentApoapsis =
                if timeToApoapsis < 600.<s> then
                    deployOrbit.Period.As<s>() + timeToApoapsis + now
                else
                    timeToApoapsis + now

            // Move to Tdeploy - 10min
            mission.WarpTo(nextDeploymentApoapsis - 600.<s>)
            
            deployNextSatellite satNum
        
    /// Deorbit deployment vessel
    let phase4 () =
        printfn "Deorbiting deployment vessel"
        let _circ: Node = Maneuver.Orbital.addHohmannNode mission Maneuver.Periapsis 10_000.<m>
        Maneuver.Execution.executeNext mission 30.<s>

    phase1 ()
    phase2 ()
    phase3 ()
    phase4 ()