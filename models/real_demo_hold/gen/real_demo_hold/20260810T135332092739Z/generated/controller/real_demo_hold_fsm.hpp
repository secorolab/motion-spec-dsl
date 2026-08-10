/*
 * This is an auto-generated file. Do not edit it directly.
 *
 * FSM: real_demo_hold_fsm
 * FSM Description: Hold the pose the arm started in, until the run is stopped.
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "real_demo_hold_fsm.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, real_demo_hold_fsm::E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, real_demo_hold_fsm::E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};
    struct fsm_nbx *fsm = real_demo_hold_fsm::create_fsm();
    if (!fsm) return 1;

    while (true) {
        produce_event(fsm->eventData, real_demo_hold_fsm::E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm->eventData, &userData);
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
    }

    real_demo_hold_fsm::destroy_fsm(fsm);
    return 0;
}

 * -----------------------------------------------------
 */

#ifndef REAL_DEMO_HOLD_FSM_HPP
#define REAL_DEMO_HOLD_FSM_HPP

#include "coord2b/types/fsm.h"
#include "coord2b/types/event_loop.h"
#include <new>


namespace real_demo_hold_fsm {

struct fsm_nbx * create_fsm();
void destroy_fsm(struct fsm_nbx * fsm);

// sm states
enum e_states {
    S_START = 0,
    S_HOLD,
    S_DONE,
    NUM_STATES
};

/// This FSM's own IRI; the tables below name its parts.
static constexpr const char * FSM_URI = "https://secorolab.github.io/models/real-demo-hold/fsm/real_demo_hold_fsm";

static constexpr const char * STATE_URIS[NUM_STATES] = {
    "https://secorolab.github.io/models/real-demo-hold/fsm/S_START",
    "https://secorolab.github.io/models/real-demo-hold/fsm/S_HOLD",
    "https://secorolab.github.io/models/real-demo-hold/fsm/S_DONE",
};

// sm events
enum e_events {
    E_STEP = 0,
    NUM_EVENTS
};

static constexpr const char * EVENT_URIS[NUM_EVENTS] = {
    "https://secorolab.github.io/models/real-demo-hold/fsm/E_STEP",
};

// sm transitions
enum e_transitions {
    T_START_HOLD = 0,
    NUM_TRANSITIONS
};

static constexpr const char * TRANSITION_URIS[NUM_TRANSITIONS] = {
    "https://secorolab.github.io/models/real-demo-hold/fsm/T_START_HOLD",
};

// sm reactions
enum e_reactions {
    R_STEP_START = 0,
    NUM_REACTIONS
};

static constexpr const char * REACTION_URIS[NUM_REACTIONS] = {
    "https://secorolab.github.io/models/real-demo-hold/fsm/R_STEP_START",
};

inline struct fsm_nbx * create_fsm() {

    struct fsm_nbx * fsm   = new (std::nothrow) fsm_nbx{
        .numReactions      = NUM_REACTIONS,
        .numTransitions    = NUM_TRANSITIONS,
        .numStates         = NUM_STATES,
        .states            = nullptr,
        .startStateIndex   = S_START,
        .endStateIndex     = S_DONE,
        .currentStateIndex = S_START,
        .eventData         = nullptr,
        .reactions         = nullptr,
        .transitions       = nullptr
    };
    if (!fsm) return nullptr;

    // sm states
    struct state * states = new (std::nothrow) state[NUM_STATES]{
        {.name = "S_start"},
        {.name = "S_hold"},
        {.name = "S_done"}
    };

    // sm transition table
    struct transition * transitions = new (std::nothrow) transition[NUM_TRANSITIONS]{
        {
            .startStateIndex = S_START,
            .endStateIndex   = S_HOLD,
        }
    };

    // sm reaction table
    struct event_reaction * reactions = new (std::nothrow) event_reaction[NUM_REACTIONS]{
        {
            .conditionEventIndex = E_STEP,
            .transitionIndex     = T_START_HOLD,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        }};

    if (!states || !transitions || !reactions) {
        delete[] states;
        delete[] transitions;
        delete[] reactions;
        delete fsm;
        return nullptr;
    }

    for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
        if (reactions[i].numFiredEvents > 0 && !reactions[i].firedEventIndices) {
            for (unsigned int j = 0; j < NUM_REACTIONS; ++j) {
                delete[] reactions[j].firedEventIndices;
            }
            delete[] reactions;
            delete[] transitions;
            delete[] states;
            delete fsm;
            return nullptr;
        }
    }

    // sm event data
    struct events * eventData = new (std::nothrow) events{};
    _Bool * currentEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    _Bool * futureEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    if (!eventData || !currentEvents || !futureEvents) {
        delete[] states;
        delete[] transitions;
        if (reactions) {
            for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
                delete[] reactions[i].firedEventIndices;
            }
        }
        delete[] reactions;
        delete[] currentEvents;
        delete[] futureEvents;
        delete eventData;
        delete fsm;
        return nullptr;
    }
    eventData->numEvents     = NUM_EVENTS;
    eventData->currentEvents = currentEvents;
    eventData->futureEvents  = futureEvents;

    // sm fsm struct
    fsm->states      = states;
    fsm->eventData   = eventData;
    fsm->reactions   = reactions;
    fsm->transitions = transitions;

    return fsm;
}

inline void destroy_fsm(struct fsm_nbx * fsm) {
    if (!fsm) return;
    if (fsm->reactions) {
        for (unsigned int i = 0; i < fsm->numReactions; ++i) {
            delete[] fsm->reactions[i].firedEventIndices;
            fsm->reactions[i].firedEventIndices = nullptr;
            fsm->reactions[i].numFiredEvents = 0;
        }
    }
    if (fsm->eventData) {
        delete[] fsm->eventData->currentEvents;
        delete[] fsm->eventData->futureEvents;
        delete fsm->eventData;
        fsm->eventData = nullptr;
    }
    delete[] fsm->reactions;
    delete[] fsm->transitions;
    delete[] fsm->states;
    delete fsm;
}

} // namespace real_demo_hold_fsm

#endif // REAL_DEMO_HOLD_FSM_HPP