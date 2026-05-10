// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef TWO_DIMENSIONAL_TORUS_SWITCH
#define TWO_DIMENSIONAL_TORUS_SWITCH

#include <cstdint>
#include "eventlist.h"
#include "switch.h"

typedef enum {
    DIMENSION_ORDERED = 0,
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
                              uint32_t id,
                              torus_routing_strategy routing_strategy);

    // TODO: introduce switch latency
    virtual void receivePacket(Packet& pkt) override;
    virtual Route* getNextHop(Packet& pkt) override;

private:
    uint32_t _id;
    string _name;
    torus_routing_strategy _routing_strategy;

    static uint32_t id;
};

#endif  //! TWO_DIMENSIONAL_TORUS_SWITCH
