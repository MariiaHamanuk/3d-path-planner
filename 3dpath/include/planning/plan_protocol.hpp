#pragma once

#include "core/communication.hpp"

#include <concepts>
#include <cstdint>

/// Message contract between the local and the global planner (see GlobalPlanningStage and
/// LocalPlanningStage in stages.hpp):
///   - the local stage sends at most one request at a time and numbers them with `id`;
///   - the global stage answers every request exactly once, echoing it in `request_id`;
///   - the local stage drops answers to requests older than its newest one.
namespace path3d {

/// A stamped replanning request with a writable numeric `id`.
template <class R>
concept PlanRequest = Stamped<R> && requires(R& request) {
  { request.id } -> std::convertible_to<std::uint64_t>;
  request.id = std::uint64_t{};
};

/// A stamped plan with a writable `request_id` naming the request it answers.
template <class P>
concept PlanAnswer = Stamped<P> && requires(P& plan) {
  { plan.request_id } -> std::convertible_to<std::uint64_t>;
  plan.request_id = std::uint64_t{};
};

}
