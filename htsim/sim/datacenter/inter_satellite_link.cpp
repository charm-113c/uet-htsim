// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "inter_satellite_link.h"
#include <cmath>
#include <cstdint>
#include <sstream>
#include "config.h"
#include "eventlist.h"
#include "network.h"
#include "pipe.h"

#define PI 3.14159265358979323846
// Gravitational constant
#define G_CONST 6.67408e-11
// Earth's radius in m, needed to compute latencies
#define R_EARTH_M 6378000
#define M_EARTH_KG 5.9722e24
// Speed of light in mps
#define C_MPS 299792458
// LEO satellite speed in mps
#define V_SAT_MPS 7500

#define SQUARE(x) (x * x)

InterSatelliteLink::InterSatelliteLink(uint32_t constellation_n,
                                       uint32_t constellation_m,
                                       ISL_Type isl_type,
                                       uint32_t orbital_inclination,
                                       uint32_t altitude)
    : Pipe(0, EventList::getTheEventList()) {
    _constellation_n = constellation_n;
    _constellation_m = constellation_m;

    _isl_type = isl_type;
    _orbital_inclination_in_deg = orbital_inclination;
    _altitude_m = altitude;

    if (_isl_type == INTRAPLANE) {
        _latency = _compute_intraplane_latency_ps();
    } else {
        // Latency is variable, so we set average as latency
        // Use same formula as in TwoDimensionalTorusTopology
        double factor1 = sqrt(2 * (1 - cos(2 * PI / _constellation_n)));
        double factor2 =
            ellint_2l((1 - SQUARE(cos(_orbital_inclination_in_deg * PI / 180))), PI / 2);

        double avg_dist = (2 / PI) * (_altitude_m + R_EARTH_M) * factor1 * factor2;
        double latency_s = avg_dist / C_MPS;
        _latency = timeFromSec(latency_s);
    }

    stringstream ss;
    ss << "InterSatelliteLink(" << timeAsUs(_latency) << "us";
    _nodename = ss.str();
}

/*
 * This inherited method works exactly as it does in Pipe, the only modification being that delay is
 * computed according to the ISL_Type.
 */
void InterSatelliteLink::receivePacket(Packet& pkt) {
    simtime_picosec delay;
    if (_isl_type == INTRAPLANE) {
        delay = _compute_intraplane_latency_ps();
    } else {
        delay = _compute_interplane_latency_ps(eventlist().now());
    }

    if (_count == 0) {
        /* no packets currently inflight; need to notify the eventlist
           we've an event pending */
        eventlist().sourceIsPendingRel(*this, delay);
    }

    _count++;
    if (_count == _size) {
        _inflight_v.resize(_size * 2);
        if (_next_insert < _next_pop) {
            //   456789*123
            // NI *, NP 1
            for (int i = 0; i < _next_insert; i++) {
                // move 4-9 into new space
                _inflight_v.at(_size + i) = _inflight_v.at(i);
            }
            _next_insert += _size;
        } else {
            // 123456789*
            // nothing to do
        }
        _size += _size;
    }
    _inflight_v[_next_insert].time = eventlist().now() + delay;
    _inflight_v[_next_insert].pkt = &pkt;
    _next_insert = (_next_insert + 1) % _size;
}

/*
 * This method computes intraplane distance, the distance between two neighbouring satellites on the
 * same horizontal plane. Intraplane distance between satellites is fixed, so no input parameters
 * needed. The formula used assumes satellites are spaced evenly along their plane, orbiting around
 * a perfectly spherical Earth. It also assumes inter-satellite links use lasers which travel at the
 * speed of light in a vaccum.
 * Leveraging this, it returns the intraplane latency between two neighbouring satellites.
 */
simtime_picosec InterSatelliteLink::_compute_intraplane_latency_ps() {
    // Distance formula used:
    // d_m = (r_earth + altitude) * sqrt( 2*(1 - cos(2pi/_m)) )
    // Compute all in m/s, return conversion to picosec

    double r = R_EARTH_M + _altitude_m;
    double factor = sqrt(2 * (1 - cos(2 * PI / _constellation_m)));
    double dist = r * factor;

    double latency_s = dist / C_MPS;
    return timeFromSec(latency_s);
}

/*
 * This method computes the distance between two neighbouring satellites in adjacent orbital planes,
 * i.e. one satellite to the north and one to the south. The trajectory of two such satellites, when
 * traced, naturally form an elliptic shape as each plane varies in trajectory from the others.
 *
 * On top of the assumptions made for intraplane latency, we also assume satellites' speed to be
 * 27,000kmph, or 7500mps. Note that since the trajectories are orbital, the distance between two
 * satellites varies in the same way for each orbital period.
 * Thus, the distance and subsequently the latency betweenm neighbouring satellites depends on the
 * time during the orbital period.
 *
 * The formula used for distance can be found in the paper "QoS-Aware Resource Placement For LEO
 * Satellite Edge Computing" by T. Pfandzelter and D. Bermbach.
 */
simtime_picosec InterSatelliteLink::_compute_interplane_latency_ps(simtime_picosec t_now) {
    // Oh boy...

    // Compute orbital period:
    // T = 2pi * sqrt((alt + r_earth)^3 / G_const * M_earth)
    double orb_period = 2 * PI * sqrt(pow(_altitude_m + R_EARTH_M, 3) / G_CONST * M_EARTH_KG);

    double factor1 = sqrt(2 * (1 - cos(2 * PI / _constellation_n)));
    double factor2 = sqrt(SQUARE(cos(2 * PI * timeAsSec(t_now) / orb_period)) +
                          SQUARE(cos(_orbital_inclination_in_deg * PI / 180)) *
                              SQUARE(2 * PI * timeAsSec(t_now) / orb_period));
    double dist_m = (_altitude_m + R_EARTH_M) * factor1 * factor2;
    double latency_s = dist_m / C_MPS;

    return timeFromSec(latency_s);
}
