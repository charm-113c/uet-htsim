// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef INTERSATELLITE_LINK
#define INTERSATELLITE_LINK

#include <cstdint>
#include "pipe.h"

/**
 * This class is made primarily to work with the 2D torus topology, when the latter is used to
 * simulate computing satellite constellations. As such, it extends Pipe, but modifies the way it
 * computes the delay for doNextEvent().
 * In short, it is meant to simulate how the distance between two satellites --and subsequently, the
 * delay in communication-- is variable rather than fixed. It can thus be seen as a variable-delay
 * Pipe.
 * In all other aspects, it is identical to Pipe.
 *
 * Note: since the Pipe class' _delay field is private and the delay() method isn't virtual,
 * the delay returned by this class in the logs will be fixed and *will not* reflect delay
 * variability.
 */
class InterSatelliteLink : public Pipe {
public:
    //! Intraplane essentially means "horizontal" links: this corresponds to links between
    //! satellites of the same plane, and as those always maintain the same distance, delay is
    //! actually constant.
    //! This isn't the case between interplane satellites, as the distance between them and thus the
    //! length of these "vertical" Pipes changes over time.
    enum ISL_Type {
        INTRAPLANE,
        INTERPLANE,
    };

    InterSatelliteLink(ISL_Type isl_type, uint32_t orbital_inclination, uint32_t _altitude);

    void receivePacket(Packet& pkt) override;  // inherited from PacketSink
    void doNextEvent() override;               // inherited from EventSource

private:
    uint32_t _orbital_inclination_in_deg;
    uint32_t _altitude;
};

#endif  //! INTERSATELLITE_LINK
