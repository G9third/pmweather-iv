package com.g9third.pmweatheriv.physics;

/** Runtime bridge exposing one native-IV ground vehicle's PMWeather wind state to movement hooks. */
public interface GroundVehicleWindStateAccess {
    GroundVehicleWind.State pmweatherIv$getGroundWindState();
}
