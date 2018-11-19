//
// Created by Lucas Letournel on 26/09/18.
//

#include "FrGeographicServices.h"

#include "GeographicLib/LocalCartesian.hpp"
#include <GeographicLib/MagneticModel.hpp>
#include <frydom/core/FrVector.h>


/**
 * In GeographicServices, the cartesian reference frame depends on the frame convention given (NED/NWU).
 * In GeographicLib, the cartesian coordinates is defined as ENU (x=East, y= North, z=Up).
 */

namespace frydom {

    FrGeographicCoord::FrGeographicCoord(const Position geoPos) {
        m_latitude = geoPos.GetX();
        m_longitude = geoPos.GetY();
        m_elevation = geoPos.GetZ();
    }

    FrGeographicCoord::FrGeographicCoord() {
        m_latitude = 0.;
        m_longitude = 0.;
        m_elevation = 0.;
    }

    FrGeographicCoord::FrGeographicCoord(double lat, double lon, double h) {
        m_latitude = lat;
        m_longitude = lon;
        m_elevation = h;
    }

    FrGeographicServices::FrGeographicServices() {
        GeographicLib::LocalCartesian();
    }

    GeographicLib::LocalCartesian FrGeographicServices::GetGeoLib() const {
        return m_LocalCartesian;
    }

    void FrGeographicServices::SetGeographicOrigin(const double lat0, const double lon0, const double h0) {
        m_LocalCartesian.Reset(lat0, lon0, h0);
    }

    void FrGeographicServices::GeoToCart(double lat, double lon, double h,
                                         double &x, double &y, double &z, FRAME_CONVENTION fc) {
        m_LocalCartesian.Forward(lat, lon, h, y, x, z);
        if (IsNED(fc)) z= -z;
        else y = -y;
    }

    Position FrGeographicServices::GeoToCart(const FrGeographicCoord& geoCoord, FRAME_CONVENTION fc) {
        double x,y,z;
        GeoToCart(geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetElevation(), y, x, z, fc);
        return {x,y,z};
    }

    Position FrGeographicServices::GeoToCart(double lat, double lon, double h, FRAME_CONVENTION fc) {
        return GeoToCart(FrGeographicCoord(lat,lon,h),fc);
    }

    void FrGeographicServices::GeoToCart(const FrGeographicCoord& geoCoord, Position& cartPos, FRAME_CONVENTION fc){
        cartPos = GeoToCart(geoCoord,fc);
    }


    void FrGeographicServices::CartToGeo(double x, double y, double z,
                                         double &lat, double &lon, double &h, FRAME_CONVENTION fc) {
        double Ytemp = y, Ztemp = z;
        if (IsNED(fc)) Ztemp= -Ztemp;
        else Ytemp = -Ytemp;
        m_LocalCartesian.Reverse(Ytemp, x, Ztemp, lat, lon, h);
    }

    FrGeographicCoord FrGeographicServices::CartToGeo(const Position& cartPos, FRAME_CONVENTION fc) {
        double lat, lon, h;
        CartToGeo(cartPos.GetX(), cartPos.GetY(), cartPos.GetZ(),lat, lon, h,fc);
        return {lat,lon,h};
    }

    void FrGeographicServices::CartToGeo(const Position& cartPos, FrGeographicCoord &geoCoord, FRAME_CONVENTION fc) {
        geoCoord = CartToGeo(cartPos,fc);
    }


    double FrGeographicServices::GetDeclinationFromCart(const Position &cartPos, double year, FRAME_CONVENTION fc) {
        /// Convert the node local coordinates to geographical coordinates
        auto geoCoord = CartToGeo(cartPos, fc);

        return GetDeclinationFromGeo(geoCoord, year);
    }

    double FrGeographicServices::GetDeclinationFromCart(double x, double y, double z, double year, FRAME_CONVENTION fc) {
        return GetDeclinationFromCart(Position(x,y,z), year, fc);
    }

    double
    FrGeographicServices::GetDeclinationFromGeo(const FrGeographicCoord &geoCoord, double year) {
        /// Magnetic model loaded from _deps directory
        GeographicLib::MagneticModel magneticModel("emm2017", "../_deps/magneticmodel-src");

        /// Compute the magnetic declination
        double Bx, By, Bz, H, F, D, I;
        magneticModel(year, geoCoord.GetLatitude(), geoCoord.GetLongitude(), geoCoord.GetElevation(), Bx, By, Bz);
        GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);

        return D;
    }

    double
    FrGeographicServices::GetDeclinationFromGeo(double lat, double lon, double h, double year) {
        return GetDeclinationFromGeo(FrGeographicCoord(lat,lon,h),year);
    }







}