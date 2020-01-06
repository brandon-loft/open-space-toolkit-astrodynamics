////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @project        Open Space Toolkit ▸ Astrodynamics
/// @file           bindings/python/src/OpenSpaceToolkitAstrodynamicsPy/Trajectory/Orbit/Models/Kepler.cpp
/// @author         Lucas Brémond <lucas@loftorbital.com>
/// @license        Apache License 2.0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <OpenSpaceToolkitAstrodynamicsPy/Trajectory/Orbit/Models/Kepler/COE.cpp>

#include <OpenSpaceToolkit/Astrodynamics/Trajectory/Orbit/Models/Kepler.hpp>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void                     OpenSpaceToolkitAstrodynamicsPy_Trajectory_Orbit_Models_Kepler ( )
{

    using namespace boost::python ;

    using ostk::core::types::Real ;

    using ostk::physics::units::Length ;
    using ostk::physics::units::Derived ;
    using ostk::physics::time::Instant ;
    using ostk::physics::env::obj::Celestial ;

    using ostk::astro::trajectory::orbit::models::Kepler ;
    using ostk::astro::trajectory::orbit::models::kepler::COE ;

    scope in_Kepler = class_<Kepler, bases<ostk::astro::trajectory::orbit::Model>>("Kepler", init<const COE&, const Instant&, const Derived&, const Length&, const Real&, const Kepler::PerturbationType&>())

        .def(init<const COE&, const Instant&, const Celestial&, const Kepler::PerturbationType&>())
        .def(init<const COE&, const Instant&, const Celestial&, const Kepler::PerturbationType&, const bool>())

        .def(self == self)
        .def(self != self)

        .def(self_ns::str(self_ns::self))
        .def(self_ns::repr(self_ns::self))

        .def("isDefined", &Kepler::isDefined)

        .def("getClassicalOrbitalElements", &Kepler::getClassicalOrbitalElements)
        .def("getEpoch", &Kepler::getEpoch)
        .def("getRevolutionNumberAtEpoch", &Kepler::getRevolutionNumberAtEpoch)
        .def("getGravitationalParameter", &Kepler::getGravitationalParameter)
        .def("getEquatorialRadius", &Kepler::getEquatorialRadius)
        .def("getJ2", &Kepler::getJ2)
        .def("getPerturbationType", &Kepler::getPerturbationType)
        .def("calculateStateAt", &Kepler::calculateStateAt)
        .def("calculateRevolutionNumberAt", &Kepler::calculateRevolutionNumberAt)

        .def("StringFromPerturbationType", &Kepler::StringFromPerturbationType).staticmethod("StringFromPerturbationType")

    ;

    enum_<Kepler::PerturbationType>("PerturbationType")

        .value("No", Kepler::PerturbationType::None)
        .value("J2", Kepler::PerturbationType::J2)

    ;

    OpenSpaceToolkitAstrodynamicsPy_Trajectory_Orbit_Models_Kepler_COE() ;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////