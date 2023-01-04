////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @project        Open Space Toolkit ▸ Astrodynamics
/// @file           OpenSpaceToolkit/Astrodynamics/Flight/System/Dynamics/AtmosphericDynamics.hpp
/// @author         Vishwa Shah <vishwa@loftorbital.com>
/// @license        Apache License 2.0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __OpenSpaceToolkit_Astrodynamics_Flight_System_Dynamics_AtmosphericDynamics__
#define __OpenSpaceToolkit_Astrodynamics_Flight_System_Dynamics_AtmosphericDynamics__

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <OpenSpaceToolkit/Astrodynamics/Flight/System/SatelliteSystem.hpp>
#include <OpenSpaceToolkit/Astrodynamics/Flight/System/Dynamics.hpp>
#include <OpenSpaceToolkit/Astrodynamics/Trajectory/State.hpp>

#include <OpenSpaceToolkit/Physics/Environment/Objects/CelestialBodies/Moon.hpp>
#include <OpenSpaceToolkit/Physics/Environment/Objects/CelestialBodies/Sun.hpp>
#include <OpenSpaceToolkit/Physics/Environment/Objects/CelestialBodies/Earth.hpp>
#include <OpenSpaceToolkit/Physics/Data/Vector.hpp>
#include <OpenSpaceToolkit/Physics/Units/Derived.hpp>
#include <OpenSpaceToolkit/Physics/Units/Length.hpp>
#include <OpenSpaceToolkit/Physics/Time/Instant.hpp>
#include <OpenSpaceToolkit/Physics/Time/Duration.hpp>
#include <OpenSpaceToolkit/Physics/Environment.hpp>

#include <OpenSpaceToolkit/Mathematics/Objects/Vector.hpp>

#include <OpenSpaceToolkit/Core/Containers/Array.hpp>
#include <OpenSpaceToolkit/Core/Types/String.hpp>
#include <OpenSpaceToolkit/Core/Types/Real.hpp>
#include <OpenSpaceToolkit/Core/Types/Integer.hpp>
#include <OpenSpaceToolkit/Core/Types/Shared.hpp>

#include "../../../../../../build/thirdparty/nrlmsise-00/src/nrlmsise-00/nrlmsise-00.h"

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

using ostk::core::types::Integer ;
using ostk::core::types::Real ;
using ostk::core::types::String ;
using ostk::core::types::Shared ;
using ostk::core::ctnr::Array ;

using ostk::math::obj::Vector3d ;

using ostk::physics::Environment ;
using ostk::physics::time::Instant ;
using ostk::physics::time::Duration ;
using ostk::physics::coord::Position ;
using ostk::physics::coord::Velocity ;
using ostk::physics::coord::Frame ;
using ostk::physics::data::Vector ;
using ostk::physics::env::obj::celest::Earth ;
using ostk::physics::env::obj::celest::Moon ;
using ostk::physics::env::obj::celest::Sun ;

using ostk::astro::trajectory::State ;
using ostk::astro::flight::system::SatelliteSystem ;
using ostk::astro::flight::system::Dynamics ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief                      Defines an atmospheric model. Represents a system of differential equations that can be solved by calling the NumericalSolver class.

class AtmosphericDynamics
{

    public:

        /// @brief              Constructor
        ///
        /// @code
        ///                     Environment environment = { ... } ;
        ///                     SatelliteSystem satelliteSystem = { ... } ;
        ///                     AtmosphericDynamics atmosphericDynamics = { environment, satelliteSystem } ;
        /// @endcode
        ///
        /// @param              [in] aSatelliteSystem A satellite system
        /// @param              [in] aState A 3 DOF state

                                AtmosphericDynamics                         (   const   SatelliteSystem&            aSatelliteSystem,
                                                                                const   Environment&                anEnvironment                             ) ;

        /// @brief              Copy Constructor
        ///
        /// @param              [in] AtmosphericDynamics A satellite dynamics

                                AtmosphericDynamics                         (   const   AtmosphericDynamics&          anAtmosphericDynamics                   ) ;

        /// @brief              Destructor

        virtual                 ~AtmosphericDynamics                        ( ) ;

        /// @brief              Clone satellite dynamics
        ///
        /// @return             Pointer to cloned satellite dynamics

        virtual AtmosphericDynamics* clone                                  ( ) const ;

        /// @brief              Equal to operator
        ///
        /// @param              [in] anAtmosphericDynamics A satellite dynamics
        /// @return             True if satellite dynamics are equal

        bool                    operator ==                                 (   const   AtmosphericDynamics&          anAtmosphericDynamics                    ) const ;

        /// @brief              Not equal to operator
        ///
        /// @param              [in] anAtmosphericDynamics A satellite dynamics
        /// @return             True if satellite dynamics are not equal

        bool                    operator !=                                 (   const   AtmosphericDynamics&          anAtmosphericDynamics                    ) const ;

        /// @brief              Output stream operator
        ///
        /// @param              [in] anOutputStream An output stream
        /// @param              [in] anAtmosphericDynamics A satellite dynamics
        /// @return             A reference to output stream

        friend std::ostream&    operator <<                                 (           std::ostream&               anOutputStream,
                                                                                const   AtmosphericDynamics&        anAtmosphericDynamics                          ) ;

        /// @brief              Check if satellite dynamics is defined
        ///
        /// @return             True if satellite dynamics is defined

        bool            isDefined                                           ( ) const ;

        /// @brief              Print satellite dynamics
        ///
        /// @param              [in] anOutputStream An output stream
        /// @param              [in] (optional) displayDecorators If true, display decorators

        void            print                                               (           std::ostream&               anOutputStream,
                                                                                        bool                        displayDecorator                            =   true ) const ;

        /// @brief              Get satellite dynamics's 3 DOF State
        ///
        /// @code
        ///                     State state = satelliteDynamics.getState() ;
        /// @endcode
        ///
        /// @return             State

        Vector3d                calculateAcceleration                       (   const   State&                      aState,
                                                                                const   Real&                       f107,
                                                                                const   Real&                       f107A,
                                                                                const   Real&                       aBallisticCoefficient                         ) const ;

    private:

        Shared<const Frame>     gcrfSPtr_ ;
        SatelliteSystem         satelliteSystem_ ;
        Environment             environment_ ;

} ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
}
}
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
