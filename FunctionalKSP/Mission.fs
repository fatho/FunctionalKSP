namespace FunctionalKSP.Mission

open KRPC.Client
open KRPC.Client.Services
open KRPC.Client.Services.SpaceCenter
open System

open FunctionalKSP.Telemetry
open FunctionalKSP.Units
open System.Collections.Generic
open FunctionalKSP.Control

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

type RefCount(initial: int) =
    let mutable count = initial

    member this.AddRef(): unit =
        count <- count + 1

    /// Returns true if the last reference was removed
    member this.RemoveRef(): bool =
        if count > 0 then
            count <- count - 1
            count = 0
        else
            invalidOp "refcount is already zero"

type RefCountedStream<'a, 'b>(stream: Stream<'a>, refcount: RefCount, map: 'a -> 'b) =
    do refcount.AddRef()
    
    interface IStream<'b> with
        member this.Value with get() = stream.Get() |> map

    interface IDisposable with
        member this.Dispose() =
            if refcount.RemoveRef() then
                stream.Remove()

/// Provides a high-level interface for a single kRPC session
type Mission(conn: Connection) =
    let ksc = conn.SpaceCenter()
    let streams = new Dictionary<System.Linq.Expressions.Expression, RefCount>()
    let refCountFor (expr: System.Linq.Expressions.Expression) =
        if streams.ContainsKey(expr) then
            streams.[expr]
        else
            let refcount = new RefCount(0)
            streams.Add(expr, refcount)
            refcount


    member val SpaceCenter = ksc with get
    member val Clock = new Clock(conn, ksc) with get

    member this.UseStream<[<Measure>] 'u>(expr: System.Linq.Expressions.Expression<float32<'u>>): RefCountedStream<float32, float32<'u>> =
        let refcount = refCountFor expr
        let stream = conn.AddStream(expr)
        new RefCountedStream<float32, float32<'u>>(stream, refcount, LanguagePrimitives.Float32WithMeasure<'u>)
    
    member this.UseStream<[<Measure>] 'u>(expr: System.Linq.Expressions.Expression<float<'u>>): RefCountedStream<float, float<'u>> =
        let refcount = refCountFor expr
        let stream = conn.AddStream(expr)
        new RefCountedStream<float, float<'u>>(stream, refcount, LanguagePrimitives.FloatWithMeasure<'u>)

    member this.UseStream<'a>(expr: System.Linq.Expressions.Expression<'a>): RefCountedStream<'a, 'a> =
        let refcount = refCountFor expr
        let stream = conn.AddStream(expr)
        new RefCountedStream<'a, 'a>(stream, refcount, id)

    interface IDisposable with
        member this.Dispose() = ()