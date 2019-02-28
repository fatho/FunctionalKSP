module ANewDawn.Staging

open KRPC.Client.Services.SpaceCenter

/// Determine whether we should activate the next stage
let shouldStage (ship: Vessel) = ship.AvailableThrust < 0.1f
// TODO: detect more sophisticated staging setups, such as dropping empty tanks etc.