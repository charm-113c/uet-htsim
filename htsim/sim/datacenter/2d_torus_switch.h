// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef _TWO_DIMENSIONAL_TORUS_SWITCH_H
#define _TWO_DIMENSIONAL_TORUS_SWITCH_H

#include <cstdint>
#include "eventlist.h"
#include "switch.h"

class TwoDimensionalTorusTopology;

typedef enum {
    DIMENSION_ORDER = 0,
    MINIMAL_ADAPTIVE_ALG = 1,
} torus_routing_strategy;

/**
 * A class for 2D tori switches. Torus topologies have a large variety of routing algorithms, each
 * with their own strengths and weaknesses. As there isn't a single "canonical" routing strategy,
 * the simplest deadlock-free algorithm (dimension-ordered routing) as well as a minimal adaptive
 * one are implemented.
 */
class TwoDimensionalTorusSwitch : public Switch {
public:
    EventList* _eventList;

    TwoDimensionalTorusSwitch(EventList* eventlist,
                              TwoDimensionalTorusTopology* torus,
                              uint32_t id,
                              string name,
                              torus_routing_strategy routing_strategy);

    virtual void receivePacket(Packet& pkt) override;
    virtual Route* getNextHop(Packet& pkt) override;

    static void set_strategy(torus_routing_strategy s) { _routing_strategy = s; }
    static torus_routing_strategy _routing_strategy;

private:
    // TODO: need a callback_pipe to simulate switch latency

    TwoDimensionalTorusTopology* _torus;
};

#endif  //! _TWO_DIMENSIONAL_TORUS_SWITCH_H
