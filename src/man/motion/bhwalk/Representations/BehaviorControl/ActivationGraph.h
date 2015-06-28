/**
 * @file ActivationGraphBH
 * The graph of executed options and states.
 *
 * @author Max Risler
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ActivationGraphBH,
{
public:
  STREAMABLE(Node,
  {
  public:
    Node() = default;
    Node(const std::string& option,
                int depth,
                const std::string& state,
                int optionTime,
                int stateTime),

    (std::string) option,
    (int)(0) depth,
    (std::string) state,
    (int)(0) optionTime,
    (int)(0) stateTime,
  });

  ActivationGraph()
  {
    graph.reserve(100);
  },

  // Initialization
  graph.reserve(100);
});

inline ActivationGraph::Node::Node(const std::string& option,
                                   int depth,
                                   const std::string& state,
                                   int optionTime,
                                   int stateTime)
: option(option),
  depth(depth),
  state(state),
  optionTime(optionTime),
  stateTime(stateTime) {}
