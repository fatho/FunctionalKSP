namespace ANewDawn.Mission

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System

open ANewDawn
open ANewDawn.Units

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

/// Provides a high-level interface for a single kRPC session
type Mission(conn: Connection) =
    let ksc = conn.SpaceCenter()
    let streams = new Streams(conn)
    let clock = streams.UseStream<s>(fun () -> ksc.UT)

    member val Connection = conn with get
    member val SpaceCenter = ksc with get
    member val Streams = streams with get
    member this.UniversalTime with get() = clock.Value
    member this.ActiveVessel with get() = ksc.ActiveVessel and set vessel = ksc.ActiveVessel <- vessel
    
    /// Blockingly warp to the specified time
    member this.WarpTo(ut: float<s>) =
        if ksc.CanRailsWarpAt() then
            ksc.WarpTo(ut / 1.<s>)
        this.Tick() |> ignore

    /// Wait until the clock moved forward and return the new time
    member this.Tick(): float<s> =
        lock clock.UnderlyingStream.Condition <| fun () ->
            let currentTime = clock.Value
            while this.UniversalTime = currentTime do
                clock.UnderlyingStream.Wait()
            this.UniversalTime

    interface IDisposable with
        member this.Dispose() =
            (clock :> IDisposable).Dispose()

type Time(ut: float<s>) =
    let seconds = int ut
    let onlySeconds = seconds % 60
    let minutes = seconds / 60
    let onlyMinutes = minutes % 60
    let hours = minutes / 60
    let onlyHours = hours % 6
    let days = hours / 6
    let onlyDays = days % 426
    let years = days / 426

    member val Seconds = onlySeconds with get
    member val Minutes = onlyMinutes with get
    member val Hours = onlyHours with get
    member val Days = onlyDays with get
    member val Years = years with get
        
    member this.ToString(shortestPossible: bool) =
        if shortestPossible && (this.Years = 0) then
            if this.Days = 0 then
                sprintf "%02d:%02d:%02d" this.Hours this.Minutes this.Seconds
            else
                sprintf "%dd %02d:%02d:%02d" this.Days this.Hours this.Minutes this.Seconds
        else
            sprintf "%dy %dd %02d:%02d:%02d" this.Years this.Days this.Hours this.Minutes this.Seconds

    override this.ToString() =
        this.ToString(true)
