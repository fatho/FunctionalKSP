namespace FunctionalKSP

module Launch =
    open KRPC.Client;
    open KRPC.Client.Services.SpaceCenter
    open System.Threading
    open System
    open FunctionalKSP.Units

    [<Measure>]
    type deg
    
    type Profile = {
      /// desired altitude of apoapsis [m]
      TargetApoapsis: float<m>;
      /// compass direction of launch [°]
      LaunchAzimuth: float<deg>;
      /// flip over to the back (like the space shuttle) instead of to the front
      UpsideDown: bool;
      /// time to wait before ignition [s]
      Countdown: int;
      /// minimum altitude before starting turn [m]
      MinTurnAlt: float<m>;
      /// minimum speed before starting turn [m/s]
      MinTurnSpeed: float<m/s>;
      /// final altitude of burn
      MaxTurnAlt: float<m>;
      /// parameter describing the turn shap
      TurnExponent: float;
    }
    
    /// The Kerbin launch profile defines a 45 degree pitch at roughly 10km altitude.
    let KerbinProfile: Profile = {
        TargetApoapsis = 80000.<m>;
        LaunchAzimuth = 90.<deg>;
        UpsideDown = false;
        Countdown = 5;
        MinTurnAlt = 1000.<m>;
        MinTurnSpeed = 100.<m/s>;
        MaxTurnAlt = 70000.<m>;
        TurnExponent = 0.4
    }
    
    // Compute current turn angle as an exponentially shaped curve w.r.t altitude
    let turnPitch profile (startAlt: float<m>) (currentAlt: float<m>) =
        let upsideDownFactor = if profile.UpsideDown then -1. else 1.
        90. - upsideDownFactor * 90. * ((currentAlt - startAlt) / (profile.MaxTurnAlt - startAlt)) ** profile.TurnExponent;

    /// Determine whether we should activate the next stage
    let shouldStage (ship: Vessel) = ship.AvailableThrust < 0.1f

    let launch (ship: Vessel) (profile: Profile) =
            if ship.Situation <> VesselSituation.Landed && ship.Situation <> VesselSituation.PreLaunch
                then invalidArg "vessel" "Not grounded"
            
            Console.WriteLine "Preparing launch"

            let conn: Connection = downcast ship.connection
            let control = ship.Control
            let pilot = ship.AutoPilot
            let body = ship.Orbit.Body;
            let flight = ship.Flight();

            control.SAS <- false
            control.Throttle <- 0.f

            let launchAlt = ship.Flight().MeanAltitude
            
            // if ship flies "backwards" we need to reverse the launch angle
            let launchAzimuth = profile.LaunchAzimuth + if profile.UpsideDown then 180.<deg> else 0.<deg>

            let altitudeS = conn.AddStream(fun () -> flight.MeanAltitude)
            let airSpeedS = conn.AddStream(fun () -> flight.TrueAirSpeed)
            let apoapsisS = conn.AddStream(fun () -> ship.Orbit.ApoapsisAltitude);

            Console.WriteLine("Initial heading = {0}", flight.Heading)
            
            // TODO: countdown
            Thread.Sleep(5)
           
            pilot.TargetPitchAndHeading(90.f, flight.Heading)
            pilot.Engage()
            control.Throttle <- 1.f
            
            // Lift-off
            let stage () = control.ActivateNextStage() |> ignore
   
            Console.WriteLine "Ignition"
            stage ()

            // Go straight up until high enough and fast enough to start turning
            // (Turning to early might result in accidental collisions with launch clamps
            // or result in the vessel spinning out of control)
            while altitudeS.Get() * 1.<m> < profile.MinTurnAlt || 
                  float (airSpeedS.Get()) * 1.<m/s> < profile.MinTurnSpeed do

                if shouldStage ship then
                    Console.WriteLine "Staging!"
                    stage ()
                
                /// If we're sufficiently high, rotate the ship to the launch direction
                // (if it wasn't rotated correctly already pre-launch)
                if altitudeS.Get() > launchAlt + 25. then
                    pilot.TargetPitchAndHeading(90.f, float32 profile.LaunchAzimuth)
                    
                Thread.Sleep(100)
                
            Console.WriteLine "Starting gravity turn"

            let turnStartAlt = altitudeS.Get()

            /// Perform the gravity turn maneuver by following a pre-defined curve
            while apoapsisS.Get() * 1.<m> < profile.TargetApoapsis ||
                  altitudeS.Get() * 1.<m> < profile.MaxTurnAlt do
                if shouldStage ship then
                    Console.WriteLine "Staging!"
                    stage ()
                
                let pitch = turnPitch profile (turnStartAlt * 1.<m>) (altitudeS.Get() * 1.<m>)
                pilot.TargetPitchAndHeading(float32 pitch, float32 launchAzimuth)

                let apo = apoapsisS.Get() * 1.<m>
                control.Throttle <- if apo > profile.TargetApoapsis
                                        then 0.f
                                        else min 1.f (max 0.1f (float32 (profile.TargetApoapsis - apo) / 1000.f))

                Thread.Sleep(100)


            control.Throttle <- 0.f
            pilot.Disengage()

            Console.WriteLine "Reached space"