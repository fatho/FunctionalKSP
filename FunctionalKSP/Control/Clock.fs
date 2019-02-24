namespace FunctionalKSP.Control

open KRPC.Client.Services

open FunctionalKSP.Extensions
open FunctionalKSP.LinearAlgebra
open FunctionalKSP.Units
open FunctionalKSP.Telemetry
open System.Security.Claims
open System
open KRPC.Client

type Clock(conn: Connection, ksc: SpaceCenter.Service) =
    let timeStream = conn.UseStream<s>(fun () -> ksc.UT)

    let mutable lastTime: float<s> = 0.<s>

    member this.Time with get(): float<s> = timeStream.Value

    /// Wait until the clock moved forward and return the new time
    member this.Tick(): float<s> =
        lock timeStream.RpcStream.Condition <| fun () ->
            let mutable currentTime = timeStream.Value
            while currentTime = lastTime do
                timeStream.RpcStream.Wait()
                currentTime <- timeStream.Value
            lastTime <- currentTime
            currentTime

    interface IDisposable with
        member this.Dispose() = (timeStream :> IDisposable).Dispose()