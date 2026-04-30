// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

// Shall we start with the constructor?
#include "2d_torus_topology.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>
#include "2d_torus_switch.h"
#include "config.h"
#include "eventlist.h"
#include "inter_satellite_link.h"
#include "logfile.h"
#include "loggers.h"
#include "loggertypes.h"
#include "queue.h"
#include "switch.h"

//! Minimal constructor used to set default parameters
TwoDimensionalTorusTopology::TwoDimensionalTorusTopology(Logfile* logfile,
                                                         QueueLoggerFactory* q_log_factory,
                                                         uint32_t n,
                                                         uint32_t m,
                                                         bool is_constellation)
    : _eventlist(&EventList::getTheEventList()),
      _logfile(logfile),
      _q_logger_factory(q_log_factory),
      _n(n),
      _m(m),
      _n_virtual_channels(0),
      _queue_type(queue_type::FAIR_PRIO),
      _queue_size_b(0),
      _linkspeed_bps(0),
      _routing_strategy(MINIMAL_ADAPTIVE_ALG),
      _is_constellation(is_constellation),
      _latency(0),
      _altitude_in_m(0),
      _inclination_in_deg(0) {};

//! Full constructor
TwoDimensionalTorusTopology::TwoDimensionalTorusTopology(Logfile* logfile,
                                                         QueueLoggerFactory* q_log_factory,
                                                         uint32_t n,
                                                         uint32_t m,
                                                         uint32_t n_virtual_channels,
                                                         queue_type queue_type,
                                                         mem_b queue_size,
                                                         linkspeed_bps linkspeed,
                                                         torus_routing_strategy routing_strategy,
                                                         bool is_constellation,
                                                         simtime_picosec latency,
                                                         uint32_t altitude,
                                                         float_t inclination)
    : TwoDimensionalTorusTopology(logfile, q_log_factory, n, m, is_constellation) {
    // --- Set parameters
    set_n_virtual_channels(n_virtual_channels);
    set_queue_params(queue_type, queue_size);
    set_linkspeed(linkspeed);
    set_routing_strategy(routing_strategy);

    if (is_constellation) {
        set_constellation_params(altitude, inclination);
    } else {
        set_switch_latency(latency);
    }
    // ---

    // Allocating switches, queues, and pipes' vectors
    _switches.resize(_n, vector<Switch*>(_m));
    _ingress_queue.resize(_n, vector<Queue*>(_m));
    _egress_queue.resize(_n, vector<Queue*>(_m));
    _horizontal_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));
    _vertical_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));
    // NOTE: resize initialises vectors with values' default type, in this case nullptr
    // If things don't go right, explicitly set values to nullptr with loops here

    // Create switches, queues and pipes
    QueueLogger* queue_logger;
    uint32_t id = 0;
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            // Create switches
            _switches[i][j] = new TwoDimensionalTorusSwitch(_eventlist, id++, _routing_strategy);
            _switches[i][j]->setName("Switch_(" + ntoa(i) + ", " + ntoa(j) + ")");

            // Init Queue and their logger
            if (_q_logger_factory) {
                queue_logger = _q_logger_factory->createQueueLogger();
            } else {
                queue_logger = nullptr;
            }
            // TODO: need to handle queue creation properly, implement alloc_queue
            _ingress_queue[i][j] = alloc_queue(queue_logger);
            _ingress_queue[i][j]->setName("IngressQueue_(" + ntoa(i) + ", " + ntoa(j) + ")");
            _egress_queue[i][j] = alloc_queue(queue_logger);
            _egress_queue[i][j]->setName("IngressQueue_(" + ntoa(i) + ", " + ntoa(j) + ")");

            // Create Pipes
            for (uint32_t k = 0; k < _n_virtual_channels; k++) {
                if (is_constellation) {
                    _horizontal_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _horizontal_pipes[i][j][k]->setName("HorizontalPipe_(" + ntoa(i) + ", " +
                                                        ntoa(j) + ")");
                    _vertical_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _vertical_pipes[i][j][k]->setName("VerticalPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                      ")");
                }
            }

            // TODO: Deal with logging, is it automatic for Switches and Pipes or is a manual setup
            // needed?
        }
    }
}

// TODO: Create constructor that reads from config file

TwoDimensionalTorusTopology::~TwoDimensionalTorusTopology() {
    // TODO: clear the vectors as done in Fat Tree
}

void TwoDimensionalTorusTopology::set_n_virtual_channels(uint32_t n) {
    n == 0 ? _n_virtual_channels = 2 : _n_virtual_channels = n;
}

void TwoDimensionalTorusTopology::set_queue_params(queue_type qt, mem_b qs) {
    qt ? _queue_type = qt : _queue_type = LOSSLESS;
    // TODO: find out more about queue size, since in here they set mysterious numbers like 8, or 7.
    // Furthermore, it appears that both in Infiniband and UEC, the focus is on a credit-based flow,
    // meaning buffer size is of low relevance, as senders must wait for buffer space to be
    // available
}

void TwoDimensionalTorusTopology::set_linkspeed(linkspeed_bps ls) {
    //! Default to 100Gbps
    ls == 0 ? _linkspeed_bps = 100000000000 : _linkspeed_bps = ls;
}

void TwoDimensionalTorusTopology::set_constellation_params(uint32_t altitude, float_t inclination) {
    if (altitude <= 160000 || altitude > 2000000) {
        cout << "Set altitude is invalid, using default of 800km" << endl;
        _altitude_in_m = 800000;
    } else {
        _altitude_in_m = altitude;
    }

    if (inclination <= 0 || inclination >= 360) {
        cout << "Set orbital inclination is invalid, using default of 45 degrees" << endl;
        _inclination_in_deg = 45;
    } else {
        _inclination_in_deg = inclination;
    }
}

void TwoDimensionalTorusTopology::set_switch_latency(simtime_picosec latency) {
    _latency = latency;
}

void TwoDimensionalTorusTopology::set_routing_strategy(torus_routing_strategy rs) {
    _routing_strategy = rs;
}

Queue* TwoDimensionalTorusTopology::alloc_queue(QueueLogger* queue_logger) {
    // TODO: figure out which queues fit our use case and implement only those
    return nullptr;
}
