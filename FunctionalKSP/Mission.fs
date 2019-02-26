namespace FunctionalKSP.Mission

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System

type MissionLogLevel =
     | Debug // only relevant during development
     | Info // infrequent status updates about the mission
     | Warning // abnormal, but correctible situation detected
     | Critical // abnormal situation leading to imminent mission failure 
     
     override this.ToString() =
        match this with
        | Debug -> "DBG"
        | Info -> "INF"
        | Warning -> "WRN"
        | Critical -> "CRT"

type LogHandler = MissionLogLevel -> string -> unit

type MissionEnv = {
    conn: Connection;
    ksc: SpaceCenter.Service;
    logHandler: LogHandler
}

type Mission<'a> = { runMission: MissionEnv -> 'a }

type MissionBuilder() =
    member this.Bind<'a, 'b>(x: Mission<'a>, f: 'a -> Mission<'b>): Mission<'b> =
        { runMission = fun env -> (f (x.runMission env)).runMission env }
    member this.Delay<'a>(f: unit -> Mission<'a>): Mission<'a> = { runMission = fun env -> (f ()).runMission env }
    member this.Return<'a>(x: 'a): Mission<'a> = { runMission = fun _env -> x }
    member this.ReturnFrom<'a>(mission: Mission<'a>): Mission<'a> = mission
    member this.Using<'a, 'b when 'a :> IDisposable>(resource: 'a, user: 'a -> Mission<'b>): Mission<'b> =
        { runMission = fun env ->
            use res = resource
            (user res).runMission env
        }
    // This feels illegal coming from Haskell, but is required to make things work
    // It's a mission that does nothing
    member this.Zero(): Mission<unit> = { runMission = fun _env -> () }

type MissionAbortException(reason: string) =
    inherit Exception(sprintf "Mission aborted: %s" reason)

module Mission =
    let mission = new MissionBuilder()

    let run (logHandler: LogHandler) (conn: Connection) (m: Mission<'a>) =
        let ksc = conn.SpaceCenter()
        let env = { conn = conn; ksc = ksc; logHandler = logHandler }
        m.runMission env

    /// Construct a mission primitive
    let missionPrimitive (runPrimitive: MissionEnv -> 'a): Mission<'a> =
        { runMission = runPrimitive }

    /// Get the space center service of the current mission
    let spaceCenter = missionPrimitive <| fun env -> env.ksc
    
    /// Get the connection of the current mission
    let connection = missionPrimitive <| fun env -> env.conn

    /// Get the currently active vessel of this mission
    let activeVessel = missionPrimitive <| fun env -> env.ksc.ActiveVessel

    /// Append a message to the mission log
    let log (level: MissionLogLevel) (message: string) = missionPrimitive <| fun env -> env.logHandler level message

    /// Abort the mission
    let abort (reason: string): Mission<'a> = 
        missionPrimitive <| fun _ -> raise (new MissionAbortException(reason))

module MissionLoggers =
    let stdoutLogger (minLevel: MissionLogLevel): LogHandler = fun (level: MissionLogLevel) (message: string) ->
        if level >= minLevel then
            printfn "[%O] %s" level message