#pragma once

#include <optional>
#include "pointing.hpp"

/**
 * @brief Stores signed angular rates for the azimuth and elevation axes.
 */
struct AngularVelocity
{
    double azimuth;   ///< Degrees per second; positive follows increasing azimuth.
    double elevation; ///< Degrees per second; positive follows increasing elevation.
};

/**
 * @brief Stores the signed difference between target and antenna angles.
 * @note This is position error, not a change between consecutive targets.
 */
struct PositionError
{
    double azimuth;   ///< Wrapped azimuth error in degrees [-180, 180].
    double elevation; ///< Target elevation minus antenna elevation, in degrees.
};

/**
 * @brief Estimates target angular velocity from two consecutive target angles.
 *
 * @param current Most recent target angles, in degrees.
 * @param previous Previous target angles, in degrees.
 * @param elapsedTime Positive elapsed time between the samples, in seconds.
 * @return Signed rates in degrees per second, or std::nullopt for non-finite
 *         inputs, non-positive elapsed time, or non-finite calculated rates.
 *
 * @note Azimuth uses the shortest wrapped difference across north. Elevation
 *       uses ordinary subtraction. Finite angle ranges are not checked here.
 */
std::optional<AngularVelocity> calcAngularVelocity(
    const PointingAngles &current,
    const PointingAngles &previous,
    float elapsedTime);

/**
 * @brief Calculates target angles minus the current antenna angles.
 *
 * @param current Desired target angles, in degrees.
 * @param previous Current antenna angles, in degrees.
 * @return Signed angular errors, or std::nullopt if an input or calculated
 *         error is non-finite.
 *
 * @note Despite the parameter names, these are target and antenna positions,
 *       not consecutive target samples. Azimuth is wrapped to [-180, 180];
 *       finite input angle ranges are not checked here.
 */
std::optional<PositionError> calcPositionError(
    const PointingAngles &current,
    const PointingAngles &previous);

/**
 * @brief Reserves an interface for combining target motion and position correction.
 *
 * @param currentTarget Most recent target angles, in degrees.
 * @param previousTarget Previous target angles, in degrees.
 * @param antennaPosition Current antenna angles, in degrees.
 * @param elapsedTime Intended sample interval, in seconds.
 * @param kp Intended proportional correction gain, in inverse seconds.
 * @return No defined result yet; the implementation is pending.
 *
 * @warning The current definition is an empty non-void stub. Do not call it
 *          until it has been implemented.
 */
std::optional<AngularVelocity> calcDesiredVelocity(
    const PointingAngles &currentTarget,
    const PointingAngles &previousTarget,
    const PointingAngles &antennaPosition,
    double elapsedTime,
    double kp);
