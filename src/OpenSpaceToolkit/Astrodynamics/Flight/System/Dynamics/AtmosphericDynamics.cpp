////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @project        Open Space Toolkit ▸ Astrodynamics
/// @file           OpenSpaceToolkit/Astrodynamics/Flight/System/Dynamics/AtmosphericDynamics.cpp
/// @author         Antoine Paletta <antoine.paletta@loftorbital.com>
/// @license        Apache License 2.0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <OpenSpaceToolkit/Astrodynamics/Flight/System/Dynamics/AtmosphericDynamics.hpp>

#include <OpenSpaceToolkit/Core/Error.hpp>
#include <OpenSpaceToolkit/Core/Utilities.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace ostk
{
namespace astro
{
namespace flight
{
namespace system
{
namespace dynamics
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using ostk::physics::units::Length ;
using ostk::physics::units::Time ;
using ostk::physics::time::Scale ;
using ostk::physics::time::DateTime ;
using ostk::physics::units::Derived ;

using ostk::physics::coord::spherical::LLA ;

static const Derived::Unit GravitationalParameterSIUnit = Derived::Unit::GravitationalParameter(Length::Unit::Meter, Time::Unit::Second) ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                AtmosphericDynamics::AtmosphericDynamics    (   const   SatelliteSystem&            aSatelliteSystem,
                                                                                const   Environment&                anEnvironment                                 )
                                :   gcrfSPtr_(Frame::GCRF()),
                                    satelliteSystem_(aSatelliteSystem),
                                    environment_(anEnvironment)
{

}

                                AtmosphericDynamics::AtmosphericDynamics    (   const   AtmosphericDynamics&          anAtmosphericDynamics                       )
                                :   gcrfSPtr_(anAtmosphericDynamics.gcrfSPtr_),
                                    satelliteSystem_(anAtmosphericDynamics.satelliteSystem_),
                                    environment_(anAtmosphericDynamics.environment_)
{

}

                                AtmosphericDynamics::~AtmosphericDynamics       ( )
{

}

AtmosphericDynamics*              AtmosphericDynamics::clone                    ( ) const
{
    return new AtmosphericDynamics(*this) ;
}

bool                            AtmosphericDynamics::operator ==              (   const   AtmosphericDynamics&          anAtmosphericDynamics                          ) const
{

    if ((!this->isDefined()) || (!anAtmosphericDynamics.isDefined()))
    {
        return false ;
    }

    return (satelliteSystem_ == anAtmosphericDynamics.satelliteSystem_) ;

}

bool                            AtmosphericDynamics::operator !=              (   const   AtmosphericDynamics&          anAtmosphericDynamics                          ) const
{
    return !((*this) == anAtmosphericDynamics) ;
}

std::ostream&                   operator <<                                 (           std::ostream&               anOutputStream,
                                                                                const   AtmosphericDynamics&        anAtmosphericDynamics                          )
{

    anAtmosphericDynamics.print(anOutputStream) ;

    return anOutputStream ;

}

bool                            AtmosphericDynamics::isDefined                ( ) const
{
    return satelliteSystem_.isDefined() ;
}

void                            AtmosphericDynamics::print                  (           std::ostream&               anOutputStream,
                                                                                        bool                        displayDecorator                            ) const
{

    displayDecorator ? ostk::core::utils::Print::Header(anOutputStream, "Atmospheric Dynamics") : void () ;

    ostk::core::utils::Print::Line(anOutputStream) << "Environment:" << environment_ ;

    ostk::core::utils::Print::Separator(anOutputStream, "Satellite System") ;
    satelliteSystem_.print(anOutputStream, false) ;

    displayDecorator ? ostk::core::utils::Print::Footer(anOutputStream) : void () ;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


Vector3d                        AtmosphericDynamics::calculateAcceleration  (   const   State&                    aState,
                                                                                const   Real&                     f107,
                                                                                const   Real&                     f107A,
                                                                                const   Real&                     aBallisticCoefficient                       ) const
{

    const Instant instant = aState.getInstant() ;

    const Vector3d velocity = aState.getVelocity().accessCoordinates() ;

    nrlmsise_input input ;
    nrlmsise_output output ;
    nrlmsise_flags flags ;

    flags.switches[0] = 1 ;
    for (size_t i = 1 ; i < 24 ; ++i) {
        flags.switches[i] = 0 ;
    }

    const Integer instantYear = instant.getDateTime(Scale::UTC).getDate().getYear() ;
    const Integer instantMonth = instant.getDateTime(Scale::UTC).getDate().getMonth() ;
    const Integer instantDay = instant.getDateTime(Scale::UTC).getDate().getDay() ;

    input.doy = Duration::Between(Instant::DateTime(DateTime(instantYear, 1, 1, 0, 0, 0), Scale::UTC), instant).inDays() + Duration::Days(1.0).inDays() ;
    input.sec = Duration::Between(Instant::DateTime(DateTime(instantYear, instantMonth, instantDay, 0, 0, 0), Scale::UTC), instant).inSeconds() ;
    input.alt = aState.accessPosition().accessCoordinates().mean() ;

    const auto earthSPtr = environment_.accessCelestialObjectWithName("Earth") ; // [TBR] This is Earth specific
    
    const LLA lla = LLA::Cartesian(aState.accessPosition().accessCoordinates(), earthSPtr->getEquatorialRadius(), earthSPtr->getFlattening()) ;

    input.g_lat = lla.getLatitude().inDegrees() ;
    input.g_long = lla.getLongitude().inDegrees() ;

    input.lst = input.sec/3600.0 + input.g_long/15.0 ;
    input.f107A = f107A ;
    input.f107 = f107 ;
    input.ap = 4.0 ;

    gtd7d(&input, &flags, &output) ;

    return 0.5 * aBallisticCoefficient * velocity.norm() * output.d[5] * velocity ;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
}
}
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
