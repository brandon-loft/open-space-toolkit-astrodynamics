////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @project        Open Space Toolkit ▸ Astrodynamics
/// @file           OpenSpaceToolkit/Astrodynamics/Flight/System.hpp
/// @author         Antoine Paletta <antoine.paletta@loftorbital.com>
/// @license        Apache License 2.0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __OpenSpaceToolkit_Astrodynamics_Flight_System__
#define __OpenSpaceToolkit_Astrodynamics_Flight_System__

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <OpenSpaceToolkit/Physics/Units/Mass.hpp>

#include <OpenSpaceToolkit/Mathematics/Geometry/3D/Objects/Composite.hpp>
#include <OpenSpaceToolkit/Mathematics/Objects/Vector.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace ostk
{
namespace astro
{
namespace flight
{

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using ostk::math::obj::Vector3d ;
using ostk::math::geom::d3::objects::Composite ;

using ostk::physics::units::Mass ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief                      Defines the generic physical system that has a mass and a certain geometry that can be composed of multiple subgeometries

class System
{

    public:

        /// @brief              Constructor
        ///
        /// @code
        ///                     Mass mass = { ... } ;
        ///                     Composite composite ( ... ) ;
        ///                     System system = { mass, composite } ;
        /// @endcode
        ///
        /// @param              [in] aMass A mass
        /// @param              [in] aGeometry A geometry

                                System                                      (   const   Mass&                       aMass,
                                                                                const   Composite&                  aGeometry                                   ) ;

        /// @brief              Copy Constructor
        ///
        /// @param              [in] aSystem A system

                                System                                      (   const   System&                     aSystem                                     ) ;

        /// @brief              Destructor

        virtual                 ~System                                     ( ) ;

        /// @brief              Clone system
        ///
        /// @return             Pointer to cloned system

        System*                 clone                                       ( ) const ;

        /// @brief              Equal to operator
        ///
        /// @param              [in] aSystem A system
        /// @return             True if systems are equal

        bool                    operator ==                                 (   const   System&                     aSystem                                     ) const ;

        /// @brief              Not equal to operator
        ///
        /// @param              [in] aSystem A system
        /// @return             True if systems are not equal

        bool                    operator !=                                 (   const   System&                     aSystem                                     ) const ;

        /// @brief              Output stream operator
        ///
        /// @param              [in] anOutputStream An output stream
        /// @param              [in] aSystem A system
        /// @return             A reference to output stream

        friend std::ostream&    operator <<                                 (           std::ostream&               anOutputStream,
                                                                                const   System&                     aSystem                                     ) ;

        /// @brief              Check if system is defined
        ///
        /// @return             True if system is defined

        virtual bool            isDefined                                   ( ) const ;

        /// @brief              Print system
        ///
        /// @param              [in] anOutputStream An output stream
        /// @param              [in] (optional) displayDecorators If true, display decorators

        virtual void            print                                       (           std::ostream&               anOutputStream,
                                                                                        bool                        displayDecorator                            =   true ) const ;

        /// @brief              Get system's mass
        ///
        /// @code
        ///                     Mass mass = system.getMass() ;
        /// @endcode
        ///
        /// @return             Mass

        Mass                    getMass                                     ( ) const ;

        /// @brief              Get system's geometry
        ///
        /// @code
        ///                     Mass mass = system.getGeometry() ;
        /// @endcode
        ///
        /// @return             Composite

        Composite               getGeometry                                 ( ) const ;

    private:

        Mass                    mass_ ;
        Composite               geometry_ ;

} ;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
